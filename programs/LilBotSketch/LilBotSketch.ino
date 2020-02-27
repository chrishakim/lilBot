
#include "Wire.h"
#include "EEPROM.h"
#include <LilBot.h>

LilBot Bot;  // Instantiate

void setup(void) {
  Bot.begin(); // Required initialization before anything else
  frontObstacleHandler = frontObstacle; // Optional
  rightObstacleHandler = rightObstacle; // Optional
  leftObstacleHandler = leftObstacle;   // Optional
}

void loop(void) {
  Bot.rotate(30);  // Turn (left with a positive value, right with negative)
  Bot.emoteByNumber(random(1,22));  // Display a random emotion
  Bot.say("Hello");
  Bot.wait(5000);  // blocking delay function, which does not block the balancing
  Bot.rotate(-30);  // Turn (left with a positive value, right with negative)
  Bot.emoteByNumber(random(1,22));  // Display a random emotion
  Bot.say("Hello");
  Bot.wait(5000);  // blocking delay function, which does not block the balancing
  // Bot.go(6000);    // Go in a straight line for a number of odometry units
  // Bot.balance();   // Call this function or Bot.wait() periodically
}

// Optional
void frontObstacle(void) {
  Bot.rotate(180);  // Turn half a turn
}

// Optional
void rightObstacle(void) {
  Bot.rotate(90);  // Make a quarter turn left
}

// Optional
void leftObstacle(void) {
  Bot.rotate(-90);  // Make a quarter turn right
}
