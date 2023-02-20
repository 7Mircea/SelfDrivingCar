# Description
The project consisted of two programs. Left_Right project contains the code that runs on the Arduino. SelfDrivingCarCode contains the code that runs on RaspberryPi.

# Electric Scheme

![ElectricScheme](/Pictures/ElectricScheme.jpg)

# Algorithm

The program uses CannyEdge from OpenCV for obtaining the edges of the road marks then computes the distance from the center.

## Original Picture

![OriginalPicture](/Pictures/OriginalImage.png)

## Final Picture

![FinalPicture](/Pictures/FindingDifferenceFromCenter.png)