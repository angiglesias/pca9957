package pca9957

import (
	"errors"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"sync"

	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
)

var DefaultOpts = Opts{
	NumPixels:  8,
	Channels:   3,
	NumStripes: 1,
	Gain:       0xff,
	Freq:       physic.MegaHertz,
}

var (
	ErrOutOfBounds      = errors.New("data is out of operation range")
	ErrUninitialized    = errors.New("device is not configured")
	ErrModeNotSupported = errors.New("operation mode not supported")
)

type Opts struct {
	// Number of pixels per stripe
	NumPixels int
	// Number of chained stripes
	NumStripes int
	// Channels per pixel
	// TODO: support non rgb stripes
	Channels int
	// Frequency of operation
	Freq physic.Frequency
	// Led driver current gain setting
	Gain uint8
}

type Dev struct {
	// Inmutable
	name  string
	s     spi.Conn
	mutex sync.Mutex
	// TODO add support to GPIO stream communication
	// p          gpiostream.PinOut
	// runtime configuration
	channels   int   // channels per pixel
	numPixels  int   // pixels per stripe
	numStripes int   // number of daisy chained stripes
	currGain   uint8 // led driver current gain
	// rect       image.Rectangle // Device bounds

	// Mutable
	// b       gpiostream.BitStream // PCA9957 encoded bits; cached to reduce heap fragmentation
	pixels *image.NRGBA // PCA9957 pixels state buffer
}

func NewSPI(p spi.Port, opts *Opts) (*Dev, error) {
	const spiFreq = physic.KiloHertz
	if opts.Freq < spiFreq {
		return nil, errors.New("pca9957: expected frequency equal or greater than 1kHz")
	}
	if opts.NumStripes < 1 {
		return nil, errors.New("pca9957: minimun number of stripes is 1 or greater")
	}
	if opts.Channels != 3 {
		return nil, fmt.Errorf("pca9657: non rgb stripe configuration not supported: %w", ErrModeNotSupported)
	}
	drvChannels := opts.Channels * opts.NumPixels
	if drvChannels < 0 || drvChannels > 24 {
		return nil, errors.New("pca9957: driver can drive maximun 24 individual leds (leds * channels)")
	}
	c, err := p.Connect(opts.Freq, spi.Mode0, 8)
	if err != nil {
		return nil, err
	}

	dev := &Dev{
		name:       "pca9957{" + c.String() + "}",
		s:          c,
		numPixels:  opts.NumPixels,
		numStripes: opts.NumStripes,
		channels:   opts.Channels,
		currGain:   opts.Gain,
		pixels:     image.NewNRGBA(image.Rect(0, 0, opts.NumPixels*opts.NumStripes, 1)),
	}
	return dev, dev.Halt()
}

func (d *Dev) String() string {
	return d.name
}

// Halt will set the device on the default rest position and will also server as an init() function
func (d *Dev) Halt() (err error) {
	if d.s == nil {
		return ErrUninitialized
	}
	// strick control of the device
	d.mutex.Lock()
	defer d.mutex.Unlock()

	// set default configuration (pwm control only, no blink)
	if err = d.symWriteReg(MODE2, 0x01); err != nil {
		goto Error
	}
	// configure leds drivers control to PWM only
	if err = d.setAllLedDrivers(LedDriverCfg(PWM_ONLY, PWM_ONLY, PWM_ONLY, PWM_ONLY)); err != nil {
		goto Error
	}
	// set default current gain setting
	if err = d.symWriteReg(IREFALL, d.currGain); err != nil {
		goto Error
	}
	// switch off all led outputs
	if err = d.symWriteReg(PWMALL, 0x00); err != nil {
		goto Error
	}
	return nil
Error:
	err = fmt.Errorf("pca9957: unable to restore device config rest position: %w", err)
	return
}

func (d *Dev) All(pwm ...uint8) (err error) {
	if d.s == nil {
		return ErrUninitialized
	}

	var pwmVal uint8 = 0xFF
	if len(pwm) > 0 {
		pwmVal = pwm[0]
	}
	d.mutex.Lock()
	defer d.mutex.Unlock()

	if err = d.symWriteReg(PWMALL, pwmVal); err != nil {
		err = fmt.Errorf("pca9957: unable to set all leds on: %w", err)
	}
	return
}

func (d *Dev) ColorModel() color.Model {
	return color.NRGBAModel
}

func (d *Dev) Bounds() image.Rectangle {
	return d.pixels.Rect
}

// Blink configures operation mode to group PWM control and
// sets all leds drivers to blink at spcified frequency and PWM as ON/OFF ratio
// dutycycle = PWM/256
func (d *Dev) Blink(freq physic.Frequency, pwm uint8) (err error) {
	if d.s == nil {
		return ErrUninitialized
	}
	if freq < minFreq || freq > maxFreq {
		return ErrOutOfBounds
	}

	factor := frequencyFactor(freq)
	d.mutex.Lock()
	defer d.mutex.Unlock()

	// set led drivers to blink mode
	if err = d.setAllLedDrivers(LedDriverCfg(GROUP_CTL, GROUP_CTL, GROUP_CTL, GROUP_CTL)); err != nil {
		goto Error
	}

	if err = d.symWriteReg(MODE2, 0x21); err != nil {
		goto Error
	}
	if err = d.symWriteReg(GRPFREQ, uint8(factor)); err != nil {
		goto Error
	}
	if err = d.symWriteReg(GRPPWM, pwm); err != nil {
		goto Error
	}
	return nil
Error:
	err = fmt.Errorf("pca9957: error configuring controller blink operation mode: %w", err)
	return
}

// NoBlink disables leds drivers blink mode
func (d *Dev) NoBlink() (err error) {
	if d.s == nil {
		return ErrUninitialized
	}
	d.mutex.Lock()
	defer d.mutex.Unlock()

	if err = d.setAllLedDrivers(LedDriverCfg(PWM_ONLY, PWM_ONLY, PWM_ONLY, PWM_ONLY)); err != nil {
		goto Error
	}
	if err = d.symWriteReg(MODE2, 0x01); err != nil {
		goto Error
	}
	return nil
Error:
	err = fmt.Errorf("pca9957: error disabling controller blink operation mode: %w", err)
	return
}

const (
	maxFreq = 15260 * physic.MilliHertz
	minFreq = maxFreq / 256
)

func frequencyFactor(target physic.Frequency) int {
	// scaling to round division without using float operations
	// use bit shifting to further accelerate multiplication and division operations
	factor := int((maxFreq << 7) / target)
	return ((factor + 1<<6) >> 7) - 1
}

// Draw implements display.Drawer.
//
// Using something else than image.NRGBA is 10x slower and is not recommended.
// When using image.NRGBA, the alpha channel is ignored
//
// A back buffer is kept so that partial updates are supported, albeit the full
// LED strip is updated synchronously.
func (d *Dev) Draw(r image.Rectangle, src image.Image, sp image.Point) error {
	if r = r.Intersect(d.pixels.Rect); r.Empty() {
		return nil
	}
	srcR := src.Bounds()
	srcR.Min = srcR.Min.Add(sp)
	if dX := r.Dx(); dX < srcR.Dx() {
		srcR.Max.X = srcR.Min.X + dX
	}
	if dY := r.Dy(); dY < srcR.Dy() {
		srcR.Max.Y = srcR.Min.Y + dY
	}
	if srcR.Empty() {
		return nil
	}

	if d.s == nil {
		return ErrUninitialized
	}
	d.mutex.Lock()
	defer d.mutex.Unlock()

	return d.rasterSPIImg(r, src, srcR)
}

const (
	wordSize = 2
)

func (d *Dev) rasterSPI() error {
	// TODO support non rgb channel configurations
	update := make([]spi.Packet, 0, d.numPixels*3)

	// generate update
	for i := 0; i < d.numPixels; i++ {
		length := 2 * d.numStripes
		// color channel update
		dataR := make([]byte, length)
		dataG := make([]byte, length)
		dataB := make([]byte, length)

		// SPI packets
		pktR := spi.Packet{W: dataR}
		pktG := spi.Packet{W: dataG}
		pktB := spi.Packet{W: dataB}
		update = append(update, pktR, pktG, pktB)

		// process stripes
		for j := 0; j < d.numStripes; j++ {
			end := (d.numStripes - j) * wordSize
			start := end - wordSize

			segmentR := dataR[start:end]
			segmentG := dataG[start:end]
			segmentB := dataB[start:end]

			strpOff := j * d.numPixels
			regOffset := PCA9957Reg(i * 3)
			pix := d.pixels.NRGBAAt(i+strpOff, 0)
			segmentR[0] = byte(PWM2+regOffset) << 1
			segmentG[0] = byte(PWM1+regOffset) << 1
			segmentB[0] = byte(PWM0+regOffset) << 1
			segmentR[1] = pix.R
			segmentG[1] = pix.G
			segmentB[1] = pix.B
		}
	}
	return d.s.TxPackets(update)
}

// One will configure only one pixel of a stripe
func (d *Dev) One(stripe, pixel int, value color.Color) error {
	if pixel < 0 || pixel > d.numPixels-1 {
		return ErrOutOfBounds
	}
	if stripe < 0 || stripe > d.numStripes-1 {
		return ErrOutOfBounds
	}

	d.mutex.Lock()
	defer d.mutex.Unlock()

	// convert color to NRGBA
	val := color.NRGBAModel.Convert(value).(color.NRGBA)

	// three transactions, one per color channel
	pkts := make([]spi.Packet, 3)
	// stripes offsets
	length := 2 * (stripe + 1)
	dataR := make([]byte, length)
	dataG := make([]byte, length)
	dataB := make([]byte, length)
	pkts[0].W = dataR
	pkts[1].W = dataG
	pkts[2].W = dataB

	for j := 0; j < d.numStripes; j++ {
		end := (d.numStripes - j) * wordSize
		start := end - wordSize

		segmentR := dataR[start:end]
		segmentG := dataG[start:end]
		segmentB := dataB[start:end]
		if j != stripe {
			// NOOP
			segmentR[0], segmentR[1] = 0xff, 0xff
			segmentG[0], segmentG[1] = 0xff, 0xff
			segmentB[0], segmentB[1] = 0xff, 0xff
			continue
		}
		regOffset := PCA9957Reg(pixel * 3)
		// red
		segmentR[0] = byte(PWM2+regOffset) << 1
		segmentR[1] = val.R
		// green
		segmentG[0] = byte(PWM1+regOffset) << 1
		segmentG[1] = val.G
		// blue
		segmentB[0] = byte(PWM0+regOffset) << 1
		segmentB[1] = val.B
	}

	return d.s.TxPackets(pkts)
}

// rasterSPIimg calculates update to send to rgb stripes controllers
func (d *Dev) rasterSPIImg(rect image.Rectangle, src image.Image, srcR image.Rectangle) error {
	switch im := src.(type) {
	case *image.NRGBA:
		// fast path, no color conversion required
		start := im.PixOffset(srcR.Min.X, srcR.Min.Y)
		// srcR.Min.Y since the output display has only a single column
		end := im.PixOffset(srcR.Max.X, srcR.Min.Y)
		copy(d.pixels.Pix[4*rect.Min.X:], im.Pix[start:end])
		return d.rasterSPI()
	default:
		// Slow path.  Convert to RGBA
		b := im.Bounds()
		m := image.NewRGBA(image.Rect(0, 0, b.Dx(), b.Dy()))
		draw.Draw(m, m.Bounds(), src, b.Min, draw.Src)
		start := m.PixOffset(srcR.Min.X, srcR.Min.Y)
		// srcR.Min.Y since the output display has only a single column
		end := m.PixOffset(srcR.Max.X, srcR.Min.Y)
		// Offset into the output buffer using rect
		copy(d.pixels.Pix[4*rect.Min.X:], m.Pix[start:end])
		return d.rasterSPI()
	}
}

// setAllLedDrivers writes led driver operation mode
func (d *Dev) setAllLedDrivers(cfg uint8) (err error) {
	var (
		ledDrivers = [...]PCA9957Reg{LEDOUT0, LEDOUT1, LEDOUT2, LEDOUT3, LEDOUT4, LEDOUT5}
	)
	// write values
	for _, drvReg := range ledDrivers {
		if err = d.symWriteReg(drvReg, cfg); err != nil {
			break
		}
	}
	// extend error if operation couldn't be completed
	if err != nil {
		err = fmt.Errorf("error configuring led drivers operating mode: %w", err)
	}
	return
}

// symWriteReg will mirror instructions to all controlled stripes
// number of afected stripes can be modified using argument overrideStripes
func (d *Dev) symWriteReg(reg PCA9957Reg, val uint8, overrideStripes ...int) error {
	stripes := d.numStripes
	if len(overrideStripes) > 0 {
		stripes = overrideStripes[0]
	}
	buf := make([]byte, 2*stripes)
	data := []byte{uint8(reg << 1), val}
	for i := 0; i < stripes; i++ {
		segment := buf[i*2 : (i+1)*2]
		segment[0] = data[0]
		segment[1] = data[1]
	}
	// TODO do some logging
	if err := d.s.Tx(buf, nil); err != nil {
		return fmt.Errorf("write error %v", err)
	}
	return nil
}

// writeReg will write the specified registry value on one stripe
func (d *Dev) writeReg(reg PCA9957Reg, val uint8, stripe int) error {
	if err := d.s.Tx([]byte{uint8(reg << 1), val}, nil); err != nil {
		return fmt.Errorf("write error %v", err)
	}
	return nil
}
