## Kalman Filter Ensemble-Based Tracker

Authors : Pedro Senna (pedrosennapsc@unifei.edu.br) , Isabela Drummond and Guilherme Bastos

_________________
Dependencies 

This code was tested in linux systems with the following specifications:

Ubuntu 16.04, probabily work on Ubuntu 12.04, 14.04 or Debian Jessie (8.5)

gcc 4.8.5 or below

opencv 3.1.0

QT 5.7

_________________
Quick start guide

Open the project in QT creator and build

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
