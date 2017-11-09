/*
 * GCodeBuffer.cpp
 *
 *  Created on: 6 Feb 2015
 *      Author: David
 */

//*************************************************************************************

#include "GCodeBuffer.h"
#include "Platform.h"
#include "RepRap.h"

// Create a default GCodeBuffer
GCodeBuffer::GCodeBuffer(const char* id, MessageType mt, bool usesCodeQueue)
	: machineState(new GCodeMachineState()), identity(id), checksumRequired(false), writingFileDirectory(nullptr),
	  toolNumberAdjust(0), responseMessageType(mt), queueCodes(usesCodeQueue), binaryWriting(false)
{
	Init();
}

void GCodeBuffer::Reset()
{
	while (PopState()) { }
	Init();
}

void GCodeBuffer::Init()
{
	gcodeLineEnd = 0;
	commandLength = 0;
	readPointer = -1;
	hadLineNumber = hadChecksum = timerRunning = false;
	computedChecksum = 0;
	bufferState = GCodeBufferState::parseNotStarted;
}

void GCodeBuffer::Diagnostics(MessageType mtype)
{
	switch (bufferState)
	{
	case GCodeBufferState::parseNotStarted:
		scratchString.printf("%s is idle", identity);
		break;

	case GCodeBufferState::ready:
		scratchString.printf("%s is ready with \"%s\"", identity, Buffer());
		break;

	case GCodeBufferState::executing:
		scratchString.printf("%s is doing \"%s\"", identity, Buffer());
		break;

	default:
		scratchString.printf("%s is assembling a command", identity);
	}

	scratchString.cat(" in state(s)");
	const GCodeMachineState *ms = machineState;
	do
	{
		scratchString.catf(" %d", (int)ms->state);
		ms = ms->previous;
	}
	while (ms != nullptr);
	scratchString.cat('\n');
	reprap.GetPlatform().Message(mtype, scratchString.Pointer());
}

inline void GCodeBuffer::AddToChecksum(char c)
{
	computedChecksum ^= (uint8_t)c;
}

inline void GCodeBuffer::StoreAndAddToChecksum(char c)
{
	computedChecksum ^= (uint8_t)c;
	if (gcodeLineEnd < ARRAY_SIZE(gcodeBuffer))
	{
		gcodeBuffer[gcodeLineEnd++] = c;
	}
}

// Add a byte to the code being assembled.  If false is returned, the code is
// not yet complete.  If true, it is complete and ready to be acted upon.
bool GCodeBuffer::Put(char c)
{
	if (c != 0)
	{
		++commandLength;
	}

	if (c == 0 || c == '\n' || c == '\r')
	{
		return LineFinished();
	}

	// Process the incoming character in a state machine
	bool again;
	do
	{
		again = false;
		switch (bufferState)
		{
		case GCodeBufferState::parseNotStarted:				// we haven't started parsing yet
			switch (c)
			{
			case 'N':
			case 'n':
				hadLineNumber = true;
				AddToChecksum(c);
				bufferState = GCodeBufferState::parsingLineNumber;
				lineNumber = 0;
				break;

			case ' ':
			case '\t':
				AddToChecksum(c);
				break;

			default:
				bufferState = GCodeBufferState::parsingGCode;
				commandStart = 0;
				again = true;
				break;
			}
			break;

		case GCodeBufferState::parsingLineNumber:			// we saw N at the start and we are parsing the line number
			if (isDigit(c))
			{
				AddToChecksum(c);
				lineNumber = (10 * lineNumber) + (c - '0');
				break;
			}
			else
			{
				bufferState = GCodeBufferState::parsingWhitespace;
				again = true;
			}
			break;

		case GCodeBufferState::parsingWhitespace:
			switch (c)
			{
			case ' ':
			case '\t':
				AddToChecksum(c);
				break;

			default:
				bufferState = GCodeBufferState::parsingGCode;
				commandStart = 0;
				again = true;
				break;
			}
			break;

		case GCodeBufferState::parsingGCode:				// parsing GCode words
			switch (c)
			{
			case '*':
				declaredChecksum = 0;
				hadChecksum = true;
				bufferState = GCodeBufferState::parsingChecksum;
				break;

			case ';':
				bufferState = GCodeBufferState::discarding;
				break;

			case '(':
				AddToChecksum(c);
				bufferState = GCodeBufferState::parsingBracketedComment;
				break;

			case '"':
				StoreAndAddToChecksum(c);
				bufferState = GCodeBufferState::parsingQuotedString;
				break;

			default:
				StoreAndAddToChecksum(c);
			}
			break;

		case GCodeBufferState::parsingBracketedComment:		// inside a (...) comment
			AddToChecksum(c);
			if (c == ')')
			{
				bufferState = GCodeBufferState::parsingGCode;
			}
			break;

		case GCodeBufferState::parsingQuotedString:			// inside a double-quoted string
			StoreAndAddToChecksum(c);
			if (c == '"')
			{
				bufferState = GCodeBufferState::parsingGCode;
			}
			break;

		case GCodeBufferState::parsingChecksum:				// parsing the checksum after '*'
			if (isDigit(c))
			{
				declaredChecksum = (10 * declaredChecksum) + (c - '0');
			}
			else
			{
				bufferState = GCodeBufferState::discarding;
				again = true;
			}
			break;

		case GCodeBufferState::discarding:					// discarding characters after the checksum or an end-of-line comment
		default:
			// throw the character away
			break;
		}
	} while (again);

	return false;
}

// This is called when we are fed a null, CR or LF character.
// Return true if there is a completed command ready to be executed.
bool GCodeBuffer::LineFinished()
{
	if (gcodeLineEnd == 0)
	{
		// Empty line
		Init();
		return false;
	}

	if (gcodeLineEnd == ARRAY_SIZE(gcodeBuffer))
	{
		reprap.GetPlatform().MessageF(ErrorMessage, "G-Code buffer '%s' length overflow\n", identity);
		Init();
		return false;
	}

	gcodeBuffer[gcodeLineEnd] = 0;
	const bool badChecksum = (hadChecksum && computedChecksum != declaredChecksum);
	const bool missingChecksum = (checksumRequired && !hadChecksum && machineState->previous == nullptr);
	if (reprap.Debug(moduleGcodes) && !writingFileDirectory)
	{
		reprap.GetPlatform().MessageF(DebugMessage, "%s%s: %s\n", identity, ((badChecksum) ? "(bad-csum)" : (missingChecksum) ? "(no-csum)" : ""), gcodeBuffer);
	}

	if (badChecksum)
	{
		if (hadLineNumber)
		{
			snprintf(gcodeBuffer, ARRAY_SIZE(gcodeBuffer), "M998 P%u", lineNumber);	// request resend
		}
		else
		{
			Init();
			return false;
		}
	}
	else if (missingChecksum)
	{
		// Checksum required but none was provided
		Init();
		return false;
	}

	commandStart = 0;
	DecodeCommand();
	return true;
}

// Decode this command command and find the start of the next one on the same line.
// On entry, 'commandStart' has already been set to the address the start of where the command should be.
// On return, the state must be set to 'ready' to indicate that a command is available and we should stop adding characters.
void GCodeBuffer::DecodeCommand()
{
	// Check for a valid command letter at the start
	commandLetter = toupper(gcodeBuffer[commandStart]);
	hasCommandNumber = false;
	commandNumber = -1;
	commandFraction = -1;
	if (commandLetter == 'G' || commandLetter == 'M' || commandLetter == 'T')
	{
		parameterStart = commandStart + 1;
		const bool negative = (gcodeBuffer[parameterStart] == '-');
		if (negative)
		{
			++parameterStart;
		}
		if (isdigit(gcodeBuffer[parameterStart]))
		{
			hasCommandNumber = true;
			// Read the number after the command letter
			commandNumber = 0;
			do
			{
				commandNumber = (10 * commandNumber) + (gcodeBuffer[parameterStart] - '0');
				++parameterStart;
			}
			while (isdigit(gcodeBuffer[parameterStart]));
			if (negative)
			{
				commandNumber = -commandNumber;
			}

			// Read the fractional digit, if any
			if (gcodeBuffer[parameterStart] == '.')
			{
				++parameterStart;
				if (isdigit(gcodeBuffer[parameterStart]))
				{
					commandFraction = gcodeBuffer[parameterStart] - '0';
					++parameterStart;
				}
			}
		}

		// Find where the end of the command is. We assume that a G or M preceded by a space and not inside quotes is the start of a new command.
		bool inQuotes = false;
		bool primed = false;
		for (commandEnd = parameterStart; commandEnd < gcodeLineEnd; ++commandEnd)
		{
			const char c = gcodeBuffer[commandEnd];
			char c2;
			if (c == '"')
			{
				inQuotes = !inQuotes;
				primed = false;
			}
			else if (!inQuotes)
			{
				if (primed && ((c2 = toupper(c)) == 'G' || c2 == 'M'))
				{
					break;
				}
				primed = (c == ' ' || c == '\t');
			}
		}
	}
	else
	{
		parameterStart = commandStart;
		commandEnd = gcodeLineEnd;
	}
	bufferState = GCodeBufferState::ready;
}

// Add an entire string, overwriting any existing content and adding '\n' at the end if necessary to make it a complete line
void GCodeBuffer::Put(const char *str, size_t len)
{
	Init();
	for (size_t i = 0; i < len; i++)
	{
		if (Put(str[i]))	// if the line is complete
		{
			return;
		}
	}

	(void)Put('\n');		// because there wasn't one at the end of the string
}

// Add a null-terminated string, overwriting any existing content
void GCodeBuffer::Put(const char *str)
{
	Put(str, strlen(str));
}

void GCodeBuffer::SetFinished(bool f)
{
	if (f)
	{
		if (commandEnd < gcodeLineEnd)
		{
			// There is another command in the same line of gcode
			commandStart = commandEnd;
			DecodeCommand();
		}
		else
		{
			Init();
		}
	}
	else
	{
		bufferState = GCodeBufferState::executing;
	}
}

// Get the file position at the start of the current command
FilePosition GCodeBuffer::GetFilePosition(size_t bytesCached) const
{
	if (machineState->fileState.IsLive())
	{
		return machineState->fileState.GetPosition() - bytesCached - commandLength + commandStart;
	}
	return noFilePosition;
}

// Is 'c' in the G Code string?
// Leave the pointer there for a subsequent read.
bool GCodeBuffer::Seen(char c)
{
	bool inQuotes = false;
	for (readPointer = parameterStart; (unsigned int)readPointer < commandEnd; ++readPointer)
	{
		const char b = gcodeBuffer[readPointer];
		if (b == '"')
		{
			inQuotes = !inQuotes;
		}
		else if (!inQuotes && toupper(b) == c)
		{
			return true;
		}
	}
	readPointer = -1;
	return false;
}

// Get a float after a G Code letter found by a call to Seen()
float GCodeBuffer::GetFValue()
{
	if (readPointer >= 0)
	{
		const float result = (float) strtod(&gcodeBuffer[readPointer + 1], 0);
		readPointer = -1;
		return result;
	}

	INTERNAL_ERROR;
	return 0.0;
}

// Get a colon-separated list of floats after a key letter
const void GCodeBuffer::GetFloatArray(float a[], size_t& returnedLength, bool doPad)
{
	if (readPointer >= 0)
	{
		size_t length = 0;
		bool inList = true;
		while(inList)
		{
			if (length >= returnedLength)		// array limit has been set in here
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "GCodes: Attempt to read a GCode float array that is too long: %s\n", gcodeBuffer);
				readPointer = -1;
				returnedLength = 0;
				return;
			}
			a[length] = (float)strtod(&gcodeBuffer[readPointer + 1], 0);
			length++;
			do
			{
				readPointer++;
			} while(gcodeBuffer[readPointer] && (gcodeBuffer[readPointer] != ' ') && (gcodeBuffer[readPointer] != LIST_SEPARATOR));
			if (gcodeBuffer[readPointer] != LIST_SEPARATOR)
			{
				inList = false;
			}
		}

		// Special case if there is one entry and returnedLength requests several.
		// Fill the array with the first entry.
		if (doPad && length == 1 && returnedLength > 1)
		{
			for(size_t i = 1; i < returnedLength; i++)
			{
				a[i] = a[0];
			}
		}
		else
		{
			returnedLength = length;
		}

		readPointer = -1;
	}
	else
	{
		INTERNAL_ERROR;
		returnedLength = 0;
	}
}

// Get a :-separated list of longs after a key letter
const void GCodeBuffer::GetLongArray(long l[], size_t& returnedLength)
{
	if (readPointer >= 0)
	{
		size_t length = 0;
		bool inList = true;
		while(inList)
		{
			if (length >= returnedLength) // Array limit has been set in here
			{
				reprap.GetPlatform().MessageF(ErrorMessage, "GCodes: Attempt to read a GCode long array that is too long: %s\n", gcodeBuffer);
				readPointer = -1;
				returnedLength = 0;
				return;
			}
			l[length] = strtol(&gcodeBuffer[readPointer + 1], 0, 0);
			length++;
			do
			{
				readPointer++;
			} while(gcodeBuffer[readPointer] != 0 && (gcodeBuffer[readPointer] != ' ') && (gcodeBuffer[readPointer] != LIST_SEPARATOR));
			if (gcodeBuffer[readPointer] != LIST_SEPARATOR)
			{
				inList = false;
			}
		}
		returnedLength = length;
		readPointer = -1;
	}
	else
	{
		INTERNAL_ERROR;
		returnedLength = 0;
	}
}

// Get a string after a G Code letter found by a call to Seen().
// It will be the whole of the rest of the GCode string, so strings should always be the last parameter.
// Use the other overload of GetString to get strings that may not be the last parameter, or may be quoted.
const char* GCodeBuffer::GetString()
{
	if (readPointer >= 0)
	{
		commandEnd = gcodeLineEnd;				// the string is the remainder of the line of gcode
		const char* const result = &gcodeBuffer[readPointer + 1];
		readPointer = -1;
		return result;
	}

	INTERNAL_ERROR;
	return "";
}

// Get and copy a quoted string returning true if successful
bool GCodeBuffer::GetQuotedString(const StringRef& str)
{
	str.Clear();
	if (readPointer >= 0)
	{
		++readPointer;				// skip the character that introduced the string
		if (gcodeBuffer[readPointer] == '"')
		{
			++readPointer;
			for (;;)
			{
				char c = gcodeBuffer[readPointer++];
				if (c < ' ')
				{
					return false;
				}
				if (c == '"')
				{
					if (gcodeBuffer[readPointer++] != '"')
					{
						return true;
					}
				}
				else if (c == '\'')
				{
					if (isalpha(gcodeBuffer[readPointer]))
					{
						// Single quote before an alphabetic character forces that character to lower case
						c = tolower(gcodeBuffer[readPointer++]);
					}
					else if (gcodeBuffer[readPointer] == c)
					{
						// Two single quotes are used to represent one
						++readPointer;
					}
				}
				str.cat(c);
			}
		}
		return false;
	}

	INTERNAL_ERROR;
	return false;
}

// Get and copy a string which may or may not be quoted. If it is not quoted, it ends at the first space or control character.
bool GCodeBuffer::GetPossiblyQuotedString(const StringRef& str)
{
	if (readPointer >= 0)
	{
		if (gcodeBuffer[readPointer + 1] == '"')
		{
			return GetQuotedString(str);
		}

		commandEnd = gcodeLineEnd;				// the string is the remainder of the line of gcode
		str.Clear();
		for (;;)
		{
			++readPointer;
			const char c = gcodeBuffer[readPointer];
			if (c < ' ')
			{
				break;
			}
			str.cat(c);
		}
		str.StripTrailingSpaces();
		return !str.IsEmpty();
	}

	INTERNAL_ERROR;
	return false;
}

// This returns a pointer to the end of the buffer where a string starts.
// It is provided for legacy use, in particular in the M23
// command that sets the name of a file to be printed.  In
// preference use GetString() which requires the string to have
// been preceded by a tag letter.
// If no string was provided, it produces an error message if the string was not optional, and returns nullptr.
const char* GCodeBuffer::GetUnprecedentedString(bool optional)
{
	commandEnd = gcodeLineEnd;					// the string is the remainder of the line
	size_t i;
	char c;
	for (i = parameterStart; i < commandEnd && ((c = gcodeBuffer[i]) == ' ' || c == '\t'); ++i) { }

	if (i == commandEnd)
	{
		if (!optional)
		{
			reprap.GetPlatform().MessageF(ErrorMessage, "%c%d: String expected but not seen.\n", commandLetter, commandNumber);
		}
		return nullptr;
	}

	return &gcodeBuffer[i];
}

// Get an int32 after a G Code letter
int32_t GCodeBuffer::GetIValue()
{
	if (readPointer >= 0)
	{
		const int32_t result = strtol(&gcodeBuffer[readPointer + 1], 0, 0);
		readPointer = -1;
		return result;
	}

	INTERNAL_ERROR;
	return 0;
}

// Get an uint32 after a G Code letter
uint32_t GCodeBuffer::GetUIValue()
{
	if (readPointer >= 0)
	{
		const uint32_t result = strtoul(&gcodeBuffer[readPointer + 1], 0, 0);
		readPointer = -1;
		return result;
	}

	INTERNAL_ERROR;
	return 0;
}

// If the specified parameter character is found, fetch 'value' and set 'seen'. Otherwise leave val and seen alone.
void GCodeBuffer::TryGetFValue(char c, float& val, bool& seen)
{
	if (Seen(c))
	{
		val = GetFValue();
		seen = true;
	}
}

// If the specified parameter character is found, fetch 'value' and set 'seen'. Otherwise leave val and seen alone.
void GCodeBuffer::TryGetIValue(char c, int32_t& val, bool& seen)
{
	if (Seen(c))
	{
		val = GetIValue();
		seen = true;
	}
}

// If the specified parameter character is found, fetch 'value' and set 'seen'. Otherwise leave val and seen alone.
void GCodeBuffer::TryGetUIValue(char c, uint32_t& val, bool& seen)
{
	if (Seen(c))
	{
		val = GetUIValue();
		seen = true;
	}
}

// Try to get a float array exactly 'numVals' long after parameter letter 'c'.
// If the wrong number of values is provided, generate an error message and return true.
// Else set 'seen' if we saw the letter and value, and return false.
bool GCodeBuffer::TryGetFloatArray(char c, size_t numVals, float vals[], const StringRef& reply, bool& seen, bool doPad)
{
	if (Seen(c))
	{
		size_t count = numVals;
		GetFloatArray(vals, count, doPad);
		if (count == numVals)
		{
			seen = true;
		}
		else
		{
			reply.printf("Wrong number of values after '\''%c'\'', expected %d", c, numVals);
			return true;
		}
	}
	return false;
}

// Try to get a quoted string after parameter letter.
// If we found it then set 'seen' true and return true, else leave 'seen' alone and return false
bool GCodeBuffer::TryGetQuotedString(char c, const StringRef& str, bool& seen)
{
	if (Seen(c) && GetQuotedString(str))
	{
		seen = true;
		return true;
	}
	return false;
}

// Try to get a string, which may be quoted, after parameter letter.
// If we found it then set 'seen' true and return true, else leave 'seen' alone and return false
bool GCodeBuffer::TryGetPossiblyQuotedString(char c, const StringRef& str, bool& seen)
{
	if (Seen(c) && GetPossiblyQuotedString(str))
	{
		seen = true;
		return true;
	}
	return false;
}

// Get an IP address quad after a key letter
bool GCodeBuffer::GetIPAddress(uint8_t ip[4])
{
	if (readPointer < 0)
	{
		INTERNAL_ERROR;
		return false;
	}

	const char* p = &gcodeBuffer[readPointer + 1];
	unsigned int n = 0;
	for (;;)
	{
		char *pp;
		const unsigned long v = strtoul(p, &pp, 10);
		if (pp == p || v > 255)
		{
			readPointer = -1;
			return false;
		}
		ip[n] = (uint8_t)v;
		++n;
		p = pp;
		if (*p != '.')
		{
			break;
		}
		if (n == 4)
		{
			readPointer = -1;
			return false;
		}
		++p;
	}
	readPointer = -1;
	return n == 4;
}

// Get an IP address quad after a key letter
bool GCodeBuffer::GetIPAddress(uint32_t& ip)
{
	if (readPointer < 0)
	{
		INTERNAL_ERROR;
		return false;
	}

	uint8_t ipa[4];
	const bool ok = GetIPAddress(ipa);
	if (ok)
	{
		ip = (uint32_t)ipa[0] | ((uint32_t)ipa[1] << 8) | ((uint32_t)ipa[2] << 16) | ((uint32_t)ipa[3] << 24);
	}
	return ok;
}

// Get the original machine state before we pushed anything
GCodeMachineState& GCodeBuffer::OriginalMachineState() const
{
	GCodeMachineState *ms = machineState;
	while (ms->previous != nullptr)
	{
		ms = ms->previous;
	}
	return *ms;
}

// Push state returning true if successful (i.e. stack not overflowed)
bool GCodeBuffer::PushState()
{
	// Check the current stack depth
	unsigned int depth = 0;
	for (const GCodeMachineState *m1 = machineState; m1 != nullptr; m1 = m1->previous)
	{
		++depth;
	}
	if (depth >= MaxStackDepth + 1)				// the +1 is to allow for the initial state record
	{
		return false;
	}

	GCodeMachineState * const ms = GCodeMachineState::Allocate();
	ms->previous = machineState;
	ms->feedrate = machineState->feedrate;
	ms->fileState.CopyFrom(machineState->fileState);
	ms->lockedResources = machineState->lockedResources;
	ms->drivesRelative = machineState->drivesRelative;
	ms->axesRelative = machineState->axesRelative;
	ms->doingFileMacro = machineState->doingFileMacro;
	ms->waitWhileCooling = machineState->waitWhileCooling;
	ms->runningM501 = machineState->runningM501;
	ms->runningM502 = machineState->runningM502;
	ms->volumetricExtrusion = false;
	ms->messageAcknowledged = false;
	ms->waitingForAcknowledgement = false;
	machineState = ms;
	return true;
}

// Pop state returning true if successful (i.e. no stack underrun)
bool GCodeBuffer::PopState()
{
	GCodeMachineState * const ms = machineState;
	if (ms->previous == nullptr)
	{
		ms->messageAcknowledged = false;			// avoid getting stuck in a loop trying to pop
		ms->waitingForAcknowledgement = false;
		return false;
	}

	machineState = ms->previous;
	GCodeMachineState::Release(ms);
	return true;
}

// Return true if this source is executing a file macro
bool GCodeBuffer::IsDoingFileMacro() const
{
	return machineState->doingFileMacro;
}

// Tell this input source that any message it sent and is waiting on has been acknowledged
// Allow for the possibility that the source may have started running a macro since it started waiting
void GCodeBuffer::MessageAcknowledged(bool cancelled)
{
	for (GCodeMachineState *ms = machineState; ms != nullptr; ms = ms->previous)
	{
		if (ms->waitingForAcknowledgement)
		{
			ms->waitingForAcknowledgement = false;
			ms->messageAcknowledged = true;
			ms->messageCancelled = cancelled;
		}
	}
}

// End
