#ifndef SERIAL_H_
#define SERIAL_H_

void serialCom();
void SerialOpen(uint8_t port, uint32_t baud);
void debugmsg_append_str(const char *str);

#endif /* SERIAL_H_ */
