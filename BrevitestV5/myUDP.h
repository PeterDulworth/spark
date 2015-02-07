//----- UDP + overloading the inappropriate UDP functions of the Spark Core (REQUIRED !)


class myUDP : public UDP {
private :
	uint8_t myBuffer[128];
	int offset = 0;
public :
	virtual int beginPacket(IPAddress ip, uint16_t port){
		offset = 0;
		return UDP::beginPacket(ip, port);
	};
	virtual int endPacket(){
		return UDP::write(myBuffer, offset);
	};
	virtual size_t write(uint8_t buffer) {
		write(&buffer, 1);
		return 1;
	}
	virtual size_t write(const uint8_t *buffer, size_t size) {
		memcpy(&myBuffer[offset], buffer, size);
		offset += size;
		return size;
	}
    UDP myUDP;
};
