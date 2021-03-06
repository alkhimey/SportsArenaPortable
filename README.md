
# Sports Arena Portable
  <div align="center">
  <img src="doc/overview.jpg" width="20%">
  </div>

 This is the source code of a portable adaptation of the prize redemption game called "Sports Arena" that was released
 by Sammy USA Corporation in 1995. More info about the original can be found in the following [link](http://www.arcade-museum.com/game_detail.php?game_id=9730)
 
 The game itself is simple. A red cursor light is circling around. The player needs to push the button
 in time so that the cursor light stops on top of one of the green "target" lights.
 
  ![Example of failing a level](doc/fail.gif)
  ![Example of winning a level](doc/victory.gif)
 
 There are 16 different levels. Levels get progressively harder: higher speed, less targets
 and additional surprises.
 
 Time limit for each level is 16 seconds. The white ring, the "background",
 indicates how much time is left.
 
 An WS2812 ring, piezzo buzzer and vibration actuator are used for the interface. 

 Level logic and animation playing is preformed from the main loop.
 Intermition animations are sequential and use delay function between each step (the whole animation is run inside single main loop iteration). 

 When a level is running, the control and display is performed inside a 20hz loop.

 Sound and vibration is performed via a timer1 interrupt. The interrupt is set to 50ms intervals.
 This allows sound and vibration to be performed concurrently with the game logic and animations.
 
  <img src="doc/internals.jpg" width="50%">

