# 3D Game Pistol and movement Controler

I have been inspired by [this video](https://www.youtube.com/watch?v=Sfhd0QZ0_Gs) and it's [source repo](https://github.com/BKTRIE/M5StickC_3D-Mouse) to create simple 3D controler for first person shooting games with use of Accelerometer and Gyroscope data.

It is best to use it with some VR glasses such as TCL NXTWEAR AIR, because if you rotate with pistol controler the view goes with you. It is not comfy if you have it just with classic screen.

[Here you can see the video of the controlers in action :)](https://youtu.be/Yo82Z4Cer4w) 

I think, this concept could be used for lots of different apps. I would like to use it for example for creating virtual Drums kit. It will use two controllers on the sticks and all the drums will be virtual. Same for virtual orchestra conductor, etc.

I am sure you will find lots of another cases how to use it. Let me know about the ideas in the issues or you can message me on twitter/linkedin (@fyziktom).

## HW

I am using two [M5StickC](https://shop.m5stack.com/products/stick-c). It has integrated Accelerometer/Gyroscope MPU6886.

You can see how does it looks like in the video above. 

### Pistol Controler

The buttons are like left/right buttons of the mouse. M5StickC allows in this mode to use another two pins from HAT port for another action. I expect to add reload (like on clasic pistol - when you move cover it will press the switch connected to some GPIO26 or 36, G0 is reserved when you use BT/WIFI I think). 

### Move Controler

This is some "raw prototype", but I think it should work nice after some tuning. It simulates the keyboard keys W,S,A,D when you rotate module front,back,left,right. So, when you will place it on your belt and you curve your body front it "pressing" 'w' on BLE keyboard (same for other moves back, left and right). You can see it on the video.

I have tried to test jump, but it is not working well now.

## FW

FW is in Arudino IDE with use of [M5StickC library](https://github.com/m5stack/M5StickC/) and [BLE Mouse](https://github.com/T-vK/ESP32-BLE-Mouse) and [BLE Keyboard](https://github.com/T-vK/ESP32-BLE-Keyboard).

It is very simplified. The data are just averaged to achieve some basic filtration. It would be better to use Kalman filter to have better data. Fortunately, even now you can have lots of fun with it. 

You need to add the libraries for M5StickC and BLE Mouse/Keyboard to your Arduino IDE to be able to compile the code. Please check the tutorials in repos of libraries. 

