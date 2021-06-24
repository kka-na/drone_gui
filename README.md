# drone_gui

**catkin workspace 새로 만들거나 기존 사용중인 workspace 사용해도 무방**  
```
cd catkin_workspace/src
git clone https://github.com/kka-na/drone_gui.git
catkin_make
```

**소스 파일**  
icon folder : GUI에 사용하는 아이콘 이미지 모음  
dgui.py : GUI및 노드 subscribe 하여 display 하기 위한 코드  
dgui.ui : GUI 파일  


**실행방법**  
```
terminal #1 ) roscore
```
```
terminal #2 ) python dgui.py  
```
GUI가 열림.   
Ready : ROS node setting  
Record : Saving .bag file  
Mission1 : Drone CCTV Mode  
Mission2 : Drone Landing Mode  
Stop : All process will stop  

**실행 화면**

[![VIP – DRONE GUI test](http://img.youtube.com/vi/ptom8BGw344/0.jpg)](https://youtu.be/ptom8BGw344) 

