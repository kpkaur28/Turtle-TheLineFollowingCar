import sensor, image, time
import pyb, math,array
from pyb import Pin, Timer
from pyb import UART

# Tracks a black line. Use [(128, 255)] for a tracking a white line.
GRAYSCALE_THRESHOLD = [(220, 255)]
INA = Pin("P6",Pin.OUT_PP) #yes
INB = Pin("P9", Pin.OUT_PP) #yes
speed = 20
INA.low()
INB.high()

#Bluetooth setup
uart = UART(1, 115200)
uart.init(115200,bits=8,parity=None,stop=1,flow=0)

commands = ['AT\r\n', 'AT+NAME=NeedSleep\r\n', 'AT+PSWD:"5555"\r\n', 'AT+VERSION\r\n', 'AT+UART=115200,0,0\r\n']


#print('writing command[0]')
#uart.write(commands[0])
#print(uart.readline())

#print('writing command[1]')
#uart.write(commands[1])
#print(uart.readline())

#print('writing command[2]')
#uart.write(commands[2])
#print(uart.readline())

#print('writing command[3]')
#uart.write(commands[3])
#print(uart.readline())

#print('writing command[4]')
#uart.write(commands[4])
#print(uart.readline())


#while True:
    #time.sleep(10)
    #if uart.any():
        #a = uart.readline()
        #print(a)


# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
# to the roi near the bottom of the image and less to the next roi and so on.
ROIS = [ # [ROI, weight]
        (0, 100, 160, 20, 0.65), # You'll need to tweak the weights for your app
        (0,  50, 160, 20, 0.3), # depending on how your robot is setup.
        (0,   0, 160, 20, 0.15)
       ]

# Compute the weight divisor (we're computing this so you don't have to make weights add to 1).
weight_sum = 0
tim = Timer(2, freq=1000) # Frequency in Hz

#TODO change freq to 300 and change prescalar value
tim1 = Timer(4, freq=300) # Frequency in Hz

actual = int ((1/1000)*(tim.source_freq()/(tim.prescaler()+1)))
CameraAngle = int((0.0015*(tim1.source_freq()/(tim1.prescaler()+1))))

ch1 = tim.channel(3, Timer.PWM, pin=Pin("P4"), pulse_width_percent = 50)
ch2 = tim1.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width = 6000) #servo?
AllAngles = []

max_angle = 45
min_angle = -45

AllAngles = [x for x in range(min_angle, max_angle+1)]
LeftHalf = [y for y in range(min_angle, 0)]
RightHalf = [z for z in range(0, max_angle+1)]

timerscaler = tim1.source_freq()/(tim1.prescaler()+1)/1000
servo_pwm = {}
motor = {}
for index,angle in enumerate(AllAngles):
    servo_pwm[angle] = int((1.15 + ((1.85 - 1.15)/(len(AllAngles)-1)) * index) * timerscaler)

for m_index,m_angle in enumerate(LeftHalf):
    motor[m_angle] = int((30 + ((75 - 30)/(len(LeftHalf)-1)) * m_index) * timerscaler)

for mR_index,mR_angle in enumerate(RightHalf):
    motor[mR_angle] = int((30 - ((75 - 30)/(len(RightHalf)-1)) * mR_index) * timerscaler)

#for angle in AllAngles:
    #print(servo_pwm[angle])

for r in ROIS: weight_sum += r[4] # r[4] is the roi weight.

# Camera setup...
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # use grayscale.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(time = 2000) # Let new settings take affect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock() # Tracks FPS.



while(True):
    if(uart.any()):
        a = uart.readchar()
        #print(a)
        if(a == 115):  #break to VCC
            INA.high()
            INB.low()
            print("going")

        elif(a == 103): #go
            INA.high()
            INB.high()
            print("breaking")

        elif(a == 105): #increase speed
            speed = speed + 10
            textSpeed = str(speed)
            ch1.pulse_width_percent(speed)
            uart.write('speed increased to: ' + textSpeed)
        elif(a == 100): #decrease speeds
            speed = speed - 10
            textSpeed = str(speed)
            ch1.pulse_width_percent(speed)
            uart.write('speed decreased to: ' + textSpeed)

    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    centroid_sum = 0

    for r in ROIS:
        blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True) # r[0:4] is roi tuple.

        if blobs:
            # Find the blob with the most pixels.
            largest_blob = max(blobs, key=lambda b: b.pixels())

            # Draw a rect around the blob.
            img.draw_rectangle(largest_blob.rect(),color=0)
            img.draw_cross(largest_blob.cx(),
                           largest_blob.cy(),color=5)

            centroid_sum += largest_blob.cx() * r[4] # r[4] is the roi weight.

    center_pos = (centroid_sum / weight_sum) # Determine center of line.

    deflection_angle = 0

    # The 80 is from half the X res, the 60 is from half the Y res. The
    # equation below is just computing the angle of a triangle where the
    # opposite side of the triangle is the deviation of the center position
    # from the center and the adjacent side is half the Y res. This limits
    # the angle output to around -45 to 45. (It's not quite -45 and 45).
    deflection_angle = -math.atan((center_pos-80)/60)

    # Convert angle in radians to degrees.
    deflection_angle = math.degrees(deflection_angle)

    print("Turn Angle: %f" % deflection_angle)

    angle = max(deflection_angle, min_angle)
    angle = min(angle, max_angle)
    CameraAngle = int (deflection_angle * timerscaler)

    pulse_width_calc = int (servo_pwm[int(angle)])
    speed = int (motor[int(angle)])
    print(pulse_width_calc)

    ch1.pulse_width_percent(speed)
    ch2.pulse_width(pulse_width_calc) #SERVO
