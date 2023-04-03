import time
import ubinascii
import machine
from umqtt.simple import MQTTClient
from machine import Pin, PWM

MQTT_BROKER = '192.168.128.219'
CLIENT_ID = ubinascii.hexlify(machine.unique_id())
user = "roger"
password = "password"
MQTT_TOPIC = "client"
Table = []

# GPIO ports for the 7seg pins
segments =  (13,12,14,27,26,25,33)

#Set-up for IO pins based on our electronic architecture
led_A = Pin(13, Pin.OUT)    # number in is Output
led_B = Pin(12, Pin.OUT)    # number in is Output
led_C = Pin(14, Pin.OUT)    # number in is Output
led_D = Pin(27, Pin.OUT)    # number in is Output
led_E = Pin(26, Pin.OUT)    # number in is Output
led_F = Pin(25, Pin.OUT)    # number in is Output
led_G = Pin(33, Pin.OUT)    # number in is Output
led_DP = Pin(32, Pin.OUT)    # number in is Output
push_button1 = Pin(22, Pin.IN)  # input as table 1
push_button2 = Pin(1, Pin.IN)  # input as table 2
push_button3 = Pin(3, Pin.IN)  # input as table 3
push_button4 = Pin(21, Pin.IN)  # input as table 4
push_button5 = Pin(19, Pin.IN)  # input as table 5
push_button6 = Pin(18, Pin.IN)  # input as table 6
push_button7 = Pin(5, Pin.IN)  # input as cancelOrder

num = {' ':(0,0,0,0,0,0,0),
    '1':(0,1,1,0,0,0,0),
    '2':(1,1,0,1,1,0,1),
    '3':(1,1,1,1,0,0,1),
    '4':(0,1,1,0,0,1,1),
    '5':(1,0,1,1,0,1,1),
    '6':(1,0,1,1,1,1,1),
    'C':(0,1,1,1,0,0,1),}

def reset():
	print("Resetting...")
	for loop in range(0,7):
		led_pwm = PWM(segments[loop].value(num[' '][loop]), 5000)
		led_pwm.duty(Table)
		time.sleep(0.001)
	time.sleep(5)
	machine.reset()

def display(n):
	for loop in range(0,7):
		segments[loop].value(num[n][loop])
		led_pwm.duty(n)
		time.sleep(0.001)
	
def talker():
	table1_state = push_button1.value()
	table2_state = push_button2.value()
	table3_state = push_button3.value()
	table4_state = push_button4.value()
	table5_state = push_button5.value()
	table6_state = push_button6.value()
	cancel_state = push_button7.value()
	if table1_state == True:
		display('1')
		return Table + [1]
	elif table2_state == True:
		display('2')
		return Table + [2]
	elif table3_state == True:
		display('3')
		return Table + [3]
	elif table4_state == True:
		display('4')
		return Table + [4]
	elif table5_state == True:
		display('5')
		return Table + [5]
	elif table6_state == True:
		display('6')
		return Table + [6]
	elif cancel_state == True:
		display('C')
		return Table[:-1]
count = 0    
def main():    
	print(f"client {CLIENT_ID} to mqtt broker: {MQTT_BROKER}\n")
	mqttClient = MQTTClient(CLIENT_ID, server=MQTT_BROKER, user=user, password=password, keepalive=60)
	mqttClient.connect()
    for i in range(0,100):
			count =+1
			Table[-1] = talker()
			if (Table[-1] > 0):
				print(f"Publishing Table number :: {Table[-1]}")
				mqttClient.publish(TOPIC, str(Table[-1]).encode())
				time.sleep(3)
			mqttClient.disconnect()

    
if __name__ == "__main__":
		try:
			main()
		except OSError as e:
			print("Error: " + str(e))
			reset()
