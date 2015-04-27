require "serialport"
require 'socket'

#encoding: utf-8

#params for serial port
port_str = "/dev/ttyACM0"  #may be different for you
baud_rate = 9600
data_bits = 8
stop_bits = 1

parity = SerialPort::NONE

usb = SerialPort.new(port_str, baud_rate, data_bits, stop_bits, parity)
puts "USB Connection Established"
server  = TCPServer.new 50000
puts "Waiting for TCP Client..."
client = server.accept
puts "TCP Connection Established"

#usb.read_timeout = -1
#just read forever
sleep(1)

# Thread for waiting for TCP data
Thread.new do
	while true do
		tcpData = client.gets
		if tcpData != ""
			printf("%s", tcpData)
			usb.write tcpData
		end
	end
end

# Thread for waiting for USB data
Thread.new do
	while true do
		usbData = usb.getc
		if usbData != nil
			printf("%c", usbData)
			client.write usbData
		end
	end
end

while true do
	if gets == "q"
		break
	end
end

server.close

