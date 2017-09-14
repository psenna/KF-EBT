## Kalman Filter Ensemble-Based Tracker

Authors : Pedro Senna (pedrosennapsc@unifei.edu.br) , Isabela Drummond and Guilherme Bastos

_________________
Dependencies 

This code was tested in linux systems with the following specifications:

Ubuntu 16.04, probabily work on Ubuntu 12.04, 14.04 or Debian Jessie (8.5)

opencv 3.1.0

_________________
Quick start guide

mkdir build && cd build

cmake ..

make

Will be creator two binaries, "KF-EBT" to be used with the VOT tollkit and "KF-EBT_WebCam" that runs using a webcam.

In the KF-EBT_WebCam, select with the mouse the upper left corner and then the lower right corner of the object of interest in the image.

_________________
References

If you find it useful or use it in your research, please cite [our paper](http://urlib.net/sid.inpe.br/sibgrapi/2017/08.21.23.41).

~~~{yaml}
@InProceedings{SennaDrumBast:2017:ReEnTr,
               author = "Senna, Pedro and Drummond, Isabela Neves and Bastos, Guilherme 
                         Sousa",
                title = "Real-time ensemble-based tracker with Kalman 
                         filter",
            booktitle = "Proceedings...",
                 year = "2017",
         organization = "Conference on Graphics, Patterns and Images, 30. 
                         (SIBGRAPI)",
  conference-location = "Niter{\'o}i, RJ",
      conference-year = "Oct. 17-20, 2017",
             language = "en",
                  url = "http://urlib.net/sid.inpe.br/sibgrapi/2017/08.21.23.41"
}
~~~
_________________
Copyright (c) 2017, Pedro Senna

Permission to use, copy, modify, and distribute this software for research purposes is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

__________________
Third-party software credits

Our tracker use several different trackers, and some files are provided by others and modified to work with our method.


The files trackers/kcf/kcf.h

		      /kcf.c

		      /complexmat.hpp

Are provided by Tomáš Vojíř kcf implementation

Source: https://github.com/vojirt/kcf


The files in trackers/ASMS are provided by Tomáš Vojíř ASMS implementation

Source: https://github.com/vojirt/asms


The files trackers/CBT/consensus/common.*

				/Consensus.*

				/fastcluster/*

Are provided by Georg Nebehay CppMT implementation

Source: https://github.com/gnebehay/CppMT 
