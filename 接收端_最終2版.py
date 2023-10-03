# Example Sound Level Sketch for the Adafruit Microphone Amplifier

from machine import Pin, ADC, UART, PWM
import time
from time import sleep
import network
import machine, neopixel
from umqtt.simple import MQTTClient

#MAX9481設置
sampleWindow = 50  # Sample window width in mS (50 mS = 20Hz)
sample = 0

adc = ADC(Pin(36))  # Configure ADC with pin number 36
adc.atten(ADC.ATTN_0DB)  # Set attenuation level to 11dB
adc.width(ADC.WIDTH_12BIT)
#MAX9481設置結束

np = neopixel.NeoPixel(machine.Pin(12), 12) #LED設置於pin16

#伺服馬達設置於pin12
servo_pin = machine.Pin(16) 
pwm = machine.PWM(servo_pin)
pwm.freq(50)
#伺服馬達設置結束

#網路設置
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
while not wlan.isconnected():
    wlan.connect('Donothing_Cheng', 'Suga19930309')
    pass

def read_voltage(): #MAX9481接收數值計算
    start_millis = time.ticks_ms()  # Start of sample window
    signal_max = 0
    signal_min = 4095

    # Collect data for 50 ms
    while time.ticks_diff(time.ticks_ms(), start_millis) < sampleWindow:
        sample = adc.read()
        if sample < 4095:  # Toss out spurious readings
            if sample > signal_max:
                signal_max = sample  # Save just the max levels
            elif sample < signal_min:
                signal_min = sample  # Save just the min levels

    peak_to_peak = signal_max - signal_min  # max - min = peak-peak amplitude
    volts = (peak_to_peak * 3.3) / 4095  # Convert to volts
    bright = (int)(volts*7) #將訊號值轉成LED亮度
    num = (int)(round(bright, 1)) #將訊號值轉成LED亮燈數量
    if num < 1:
        num = 1
        bright = 5
    elif num > 12:
        num = 12
    #設定不同燈亮不同顏色
    for a in range(0, 12, 1):
        np[a] = (1*bright, 0, 3*bright)
    for j in range(num, 12, 1): #將不須亮的LED轉暗
        np[j] = (0, 0, 0)
    np.write()
    print(volts)

def sub_cb(topic, msg): #MQTT接收訊息
    while True:
        read_voltage()
        if(topic==b'A1083357_Test'):
            #print((topic, msg))
            string = str(msg)
            str2 = string[2:4]
            try:
                Num = int(str2)
                if (Num>90) :
                    Num = 90
                elif (Num<30):
                    Num = 30
            except:
                Num = 30
            pwm.duty(Num)#轉動伺服馬達
            break
def main():
    uart = UART(1, baudrate=9600)  # Configure UART with baudrate 9600
    time.sleep(1)  # Wait for UART to settle
    
    #MQTT設置
    mqClient0 = MQTTClient('Test00','140.127.218.172')
    mqClient0.set_callback(sub_cb)
    mqClient0.connect()
    mqClient0.subscribe("A1083357_Test")
    
    while True:
        #volts = read_voltage()
        mqClient0.wait_msg()

if __name__ == '__main__':
    main()


