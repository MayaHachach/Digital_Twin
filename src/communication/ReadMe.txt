sudo apt install libeigen3-dev
sudo apt install libtf2-dev

To make sure the above library were installed, you can check the folder: /usr/include
on terminal: cd /usr/include 
and then u can examine the folders and files


//explanation for transformation

Compute the Transformation (T_correction)

The transformation T_correction between the two poses is:
Tcorrection=Tcorrected⋅Todom−1
Tcorrection​=Tcorrected​⋅Todom−1​

Where:
    TcorrectedTcorrected​ is the transformation matrix from the corrected pose.
    TodomTodom​ is the transformation matrix from the odometry pose.
    Todom−1Todom−1​ is the inverse of the odometry pose transformation.

This transformation will map future odometry updates into the corrected frame.


//! important note, human correction should only be applied on objects while in static state (cant be applied on objects while theyre moving)