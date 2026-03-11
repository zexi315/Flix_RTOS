# Flix_RTOS

## 项目介绍

### **项目进行中。。。。。。。。。。**     

计划实现多种方式操控无人机

----

## 参考开源
- https://github.com/okalachev/flix  
- https://oshwhub.com/malagis/esp32-mini-plane  

---

## 开发日志

### 2025-03-08
控制放loop循环中在加入光流和TOF传感器读取后出现阻塞，导致pid参数失效;   
计划拆分代码，将控制部分放入FreeRTOS中执行。  
 
### 2025-03-10
没有备用扇叶了
计划在新扇叶到之前把esp_now遥控代码写完测试一下




