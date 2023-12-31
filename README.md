# Buttons

## About
Buttons was an autonomous, Raspberry Pi-powered robot designed for a university competition.

![Buttons](Buttons.png)

Code collaboration between Jackson Sprigg and Preet Patel. Both of us acknowledge that the code should be much cleaner. We unfortunately didn't get the chance to refactor.

## The Challenge
Buttons was required to:
1. Await packages within a designated loading zone
2. Distinguish between different types of packages and thus their delivery location (3 possibilities)
3. Navigate to the desired delivery bin while avoiding obstacles and other robots
4. Deposit the packages and return to the loading zone
5. Sustain this cycle for the duration of the competition without human interference

Watch a video of Buttons in a testing environment [here](https://clipchamp.com/watch/PpKLTU7v0G8).

## Overview
The optimisation strategies used encompass a 30 page report which, let's be honest, you probably don't care about.
So instead, I will outline some of the challenges we faced and how we overcame them:
- Localisation with low-quality sensors | Crosschecking and averaging between different sensor types
- Mapping a dynamic environment from a moving system | Implemented Vector Fields Histograms
- Efficient path planning around dynamic obstacles | Developed a custom tentacle cost function
- Building tests to monitor component health | Apparently hardcoding doesn't work when your low quality components slowly degrade and change values
- Simulating test environments | Many hours spent deterrmining real world coefficients

Overall Buttons was successful. It could delivery ~6 parcels per minute sustained for up to 4 hours (calculated battery time) and achieved the highest team score in the competition.
