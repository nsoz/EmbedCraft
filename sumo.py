from machine import Pin, ADC, Timer
import utime
#To use leaves on the line that will allow us to manipulate our code on both external data and time signatures.
#libraries we added

trig_pin = Pin(10, Pin.OUT)
echo_pin = Pin(9, Pin.IN)
#kullandığımız ultra sonik sensörler (cisim sensörleri) trig_pin fonksiyonu aracılığı ile tanıtım
#echo_pin ile sensöre gelen yankiları yani verileri toparlıyoruz

left_wheel_forward = Pin(14, Pin.OUT)
left_wheel_back = Pin(15, Pin.OUT)
right_wheel_forward = Pin(16, Pin.OUT)
right_wheel_back = Pin(17, Pin.OUT)

edge_sensor_pin = ADC(Pin(26))
#kızıl ötesi sensör kullanarak beyaz cizgiyi algılayacak olan sensörü tanıtıyorum

def read_ultrasonic():  #ultrasonik sensörden veri okuma 
    trig_pin.low()      #sensörün pinini düşük konuma getirip sinyal bekler
    utime.sleep_us(2)   #pinin low konumuna geçmesini bekle
    trig_pin.high()     #pini yüksek konuma getirir (ultra ssonik sesler yollanır companent gönderme işlemini gerçekleştirir)
    utime.sleep_us(5)   #5 micro saniye beklenir 
    trig_pin.low()      #pin tekrardan düşük konuma getirilir sinyal gönderme işleminin bittiği anlamına gelir
    
    while echo_pin.value() == 0:              #sinyal gelmediği sürece sinyal beklem konumunda kalır
        signal_off = utime.ticks_us()         #sinyalin başladığı anı kaydeder
        
    while echo_pin.value() == 1:              #sinyal ulaştığı ve devam ettiği sürece çalışır
        signal_on = utime.ticks_us()          #sinyalin bittiği anı kaydeder
        
    time_passed = signal_on - signal_off      #sinyalin başladığı andan bittiği ana kdr sinyalin var olduğu süre
    distance = (time_passed * 0.0343) / 2     #sesin havadaki hızı 0.0343 * geçen zaman / 2 (giden ses gelen ses aynı mesafe ondan doalyı / 2)
    
    return distance                           #mesafe döndürülür   

def read_edge_sensor():                      #kenar sesnösrü fonksiyonu
    edge_value = edge_sensor_pin.read_u16()  #kızıl ötesisensrden alınan veri yansıma miktarı sanırım emin değilim(!!!)okunur ve değişkne atanır
    threshold = 30000                        #beyaz zemindeki yansıma verisi ile siyah zemindeki yansıma verisinin arasındaki sınır
    if edge_value > threshold:               #eğer veri sınardan büyükse
        return "line Spoted"
    else:                                    #değilse
        return "Zemin"

def right_wheel_movement(forward : bool, speed : int):
    if forward:
        right_wheel_forward.high()
        right_wheel_back.low()
    else:
        right_wheel_forward.low()
        right_wheel_back.high()

def left_wheel_movement(forward : bool, speed : int):
    if forward:
        left_wheel_forward.high()
        left_wheel_back.low()
    else:
        left_wheel_forward.low()
        left_wheel_back.high()

while True:                                     #sürekli veri okayan sinyal bekleyen sonsuz döngü
    distance_to_object = read_ultrasonic()      #cisim sensörü
    edge_status = read_edge_sensor()            #kenar sensörü
    
    print("Mesafe (cm):", distance_to_object)
    print("Kenar durumu:", edge_status)
    
    if edge_status == "line Spoted":
        right_wheel_movement(forward = False, speed = 0)
        left_wheel_movement(forward = False, speed = 0)
    else:
        right_wheel_movement(forward = True, speed = 100)
        left_wheel_movement(forward = True, speed = 100)

    utime.sleep(0.1)
