/*
 * TelnetResponder.h
 *
 *  Created on: 14 Apr 2017
 *      Author: David
 */

#ifndef SRC_DUETNG_DUETETHERNET_TELNETRESPONDER_H_
#define SRC_DUETNG_DUETETHERNET_TELNETRESPONDER_H_

#include "NetworkResponder.h"

class TelnetResponder : public NetworkResponder
{
public:
	TelnetResponder(NetworkResponder *n);
	bool Spin() override;								// do some work, returning true if we did anything significant
	bool Accept(Socket *s, Protocol protocol) override;	// ask the responder to accept this connection, returns true if it did
	void Terminate(Protocol protocol) override;			// terminate the responder if it is serving the specified protocol

	void HandleGCodeReply(const char *reply);
	void HandleGCodeReply(OutputBuffer *reply);
	void Diagnostics(MessageType mtype) const override;

private:
	void CharFromClient(char c);
	void ProcessLine();

	bool haveCompleteLine;
	char clientMessage[GCODE_LENGTH];
	size_t clientPointer;
	uint32_t connectTime;

	static const uint32_t TelnetSetupDuration = 4000;	// ignore the first Telnet request within this duration (in ms)
};

#endif /* SRC_DUETNG_DUETETHERNET_TELNETRESPONDER_H_ */
