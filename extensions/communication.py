import json
from controller import Emitter, Receiver
from typing import List, Dict


def send_message(emitter: Emitter,
				 source_name: str,
				 message_type: str,
				 data: Dict) -> None:
	message = {
		"source": source_name,
		"type": message_type,
		"data": data
	}

	payload = json.dumps(message).encode("utf-8")
	emitter.send(payload)


def receive_all_messages(receiver: Receiver) -> List[Dict]:
	messages = []

	while receiver.getQueueLength() > 0:
		try:
			data = receiver.getString()
			# message = json.loads(data.decode("utf-8"))
			message = json.loads(data)
			messages.append(message)

		except Exception as e:
			print(f"[ERROR] Failed to parse message: {e}")

		receiver.nextPacket()


	return messages