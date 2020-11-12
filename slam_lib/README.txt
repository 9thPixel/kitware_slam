Prediction theoretically could replace registration initialization (which is performed with linear extrapolation of movement). 
However, this breaks the registration process for the car loop dataset. 
It does not change the LiDARUSA results (forest walking).
Idea : The estimation of velocity does not seem accurate enough with Kalman prediction in curves to lead to a good first estimation...
