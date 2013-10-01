tracker
=======

Particle tracking implementation for uDAQ

## installation

After installing [gcc-arm-embedded](https://launchpad.net/gcc-arm-embedded)
as well as [dfu-util](http://dfu-util.gnumonks.org/) from git,

    $ git clone --recursive git://github.com/bgamari/tracker
    $ cd tracker
    $ make
    $ sudo make program
    

## notes

Samples from the ADC are assumed to have the following channel mapping,

   0    =   stage feedback x
   1    =   stage feedback y
   2    =   stage feedback z
   3    =   PSD x sum
   4    =   PSD x difference
   5    =   PSD y sum
   6    =   PSD y difference
