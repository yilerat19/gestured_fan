# gestured_fan
At the embedded lab, we create a machine which can use gesture to control the fan.

#abstruct
In 21th century, the topic of intelligence becomes very important. In this final
project, we want to build a machine which combines Raspberry pi and Arduino together to
let us use gesture to control the 3C product.
Raspberry pi will use Raspberry Pi Camera module to detect the gesture and it will
also use its ability of calculation to distinguish different gesture. After calculating, it will send
a signal to Arduino. Arduino will use this signal to decide which IR signal it has to send. In
the end, the fan will turn on or do some function. Moreover, we use auto mode to sense the
room’s temperature and the fan will automatically wind up or down according to it.
In the future, we want to use this method as a reference to the Internet of things.
By detecting the behavior of user, 3C product can be more useful and convenient for us to
control. Not only bluetooth but also IR light are convenient ways to communicate with the
3C product. We try to build a easy prototype to fulfuill the dream about intelligent 3C
product

#report
see the report.pdf file

#demo video
https://youtu.be/tmEhdlocdy4

#code
This project has 3 parts of code

1. the auto log part:auto_execure.txt

2. for raspberry pi and its camera to catch the gesture:code_on_rpi.py

3. the IR transmission:code_on_arduino.txt
