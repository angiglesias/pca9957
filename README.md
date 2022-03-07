# Periph.io compatible SPI driver for PCA9957 led controller

NXP PCA9957 device driver to work with [periph.io](https://periph.io/)

[![PkgGoDev](https://pkg.go.dev/badge/github.com/angiglesias/pca9957)](https://pkg.go.dev/github.com/angiglesias/pca9957)

## Driver summary

The PCA9957 is a 24-channel SPI bus constant current programmable LED driver.

This device supports daisy chaining of multiple drivers to build composed led stripes. Driver will expose
the daisy chained stripes as a row of pixels as done in existing [nrzled](https://pkg.go.dev/periph.io/x/devices/v3/nrzled) driver.

## Limitations

* Driver is currently only compatible with RGB strips (max 8 pixels, asigning 3 channels per pixel)
* Driver asumes pixels in BGR configuration (in each pixel, lower channel for blue, higher channel for red and mid for green)
    * ie, for the first pixel of the stripe, channel zero would be blue, channel one would green and channel two would be red
* Driver models led stripe as one line of pixels (containing a maximun of eight color pixels)

## Datasheet

* [PCA9957](https://www.nxp.com/docs/en/data-sheet/PCA9957DS.pdf)