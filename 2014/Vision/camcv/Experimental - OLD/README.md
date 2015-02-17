Experimental Directory
======================
No, experimental does not mean cool new stuff.

This is a directory of random development efforts that were made prior to us moving to a proper source control. Much of the stuff in here won't work. That said, here's a quick description of what (I think) these files did.

camcv.c.filtered
----------------
Played with some of the hardware filters that the Pi can employ with its GPU. Appears to be disabled here. Since the GPU is otherwise useless to us, this filtering can be considered a freebie. If we can get something useful out of this DO IT. See the next file first.

camcv.c.filtered_video
----------------------
The same thing as the other filtered file, but seems to be newer. Note that my modification seems to actually be active. See line 344. I encourage you to play around with the image effects on the Pi just by using the regular raspistill command.

camcv.c.goodPointsToTrack
-------------------------
Uses OpenCV's goodFeaturesToTrack routine to get - shockingly - features to track. This works pretty OK on the Pi, though these aren't actually great feature descriptors, nor are they really useful to us at this time.

camcv.c.lessRecent
------------------
My diff tool leads me to believe that this is from late in our development efforts. Most of the stuff in the final version is here, but a couple key things are still missing.

camcv.c.LK
----------
Computes optical flow between camera images using ___ from OpenCV and the image descriptors from goodPointsToTrack. Again, this is actually pretty useless for our pylon tracking at this time - unless we can get a very good idea of how big the pylons are, which may allow us to attempt a basic 3D recovery of the scene.

camcv.c.Linsanity
-----------------
Contributions from the infamous Linsanity. Attempted to use OpenCV's adaptive thresholding. Didn't seem to work well for our purposes.

camcv.c.old
-----------
It says old. Don't touch it. It seriously have no recollection of what this is, but am too afraid to remove it.

camcv.c.moreLinsane
-------------------
Another contribution from Mr. Lin. This is from just before he joined the main development branch.

camcv.c.orange_filter
---------------------
No exciting GPU stuff here. Just does colour thresholding using OpenCV in the YUV colour space that the camera uses. Note that YUV is another way of encoding pixel colours, as opposed to something like RGB. YUV is fairly common among certain cameras. I hear.

camcv.c.serial
--------------
This is from pretty late in the development effort. This is where serial communations to the motor controller was added. It works well enough, though we may want to move to something more robust like I2C or just Ethernet.

camcv.cpp
---------
Our well intentioned but ill fated attempt to move to C++ so we could use more modern OpenCV. This has been done by someone else now, and we intend to migrate to that code.

camcv.DJ
--------
One of Duane's much appreciated additions to our development efforts. This is from failr late in the game, and is attempting to find the centre line. More to come in the project Wiki.

camcv.c.single_still
--------------------
VERY OLD. This is from when I was just starting in on this. Only gets a single frame.

camcv.c.stills_video
--------------------
VERY OLD, but slightly less so. Takes a set of stills to make a kind-of-video. "But wait! Still sets do make videos!" you may say. Well, these don't really. The camera is really just taking a set of low resolution photos, but as photos. It performed really badly.

README.md
---------
You're reading it silly!
