package gpsd

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math"
	"net"
	"strconv"
	"strings"
	"time"
)

// DefaultAddress of gpsd (localhost:2947)
const DefaultAddress = "localhost:2947"

// Filter is a gpsd entry filter function
type Filter func(interface{})

// Session represents a connection to gpsd
type Session struct {
	socket  net.Conn
	reader  *bufio.Reader
	filters map[string][]Filter
}

// Mode describes status of a TPV report
type Mode byte

const (
	// NoValueSeen indicates no data has been received yet
	NoValueSeen Mode = 0
	// NoFix indicates fix has not been required yet
	NoFix Mode = 1
	// Mode2D represents quality of the fix
	Mode2D Mode = 2
	// Mode3D represents quality of the fix
	Mode3D Mode = 3
)

type gpsdReport struct {
	Class string `json:"class"`
}

// TPVReport is a Time-Position-Velocity report
type TPVReport struct {
	Class  string    `json:"class"`
	Tag    string    `json:"tag"`
	Device string    `json:"device"`
	Mode   Mode      `json:"mode"`
	Time   time.Time `json:"time"`
	Ept    float64   `json:"ept"`
	Lat    float64   `json:"lat"`
	Lon    float64   `json:"lon"`
	Alt    float64   `json:"alt"`
	Epx    float64   `json:"epx"`
	Epy    float64   `json:"epy"`
	Epv    float64   `json:"epv"`
	Track  float64   `json:"track"`
	Speed  float64   `json:"speed"`
	Climb  float64   `json:"climb"`
	Epd    float64   `json:"epd"`
	Eps    float64   `json:"eps"`
	Epc    float64   `json:"epc"`
	Eph    float64   `json:"eph"`
}

// SKYReport reports sky view of GPS satellites
type SKYReport struct {
	Class      string      `json:"class"`
	Tag        string      `json:"tag"`
	Device     string      `json:"device"`
	Time       time.Time   `json:"time"`
	Xdop       float64     `json:"xdop"`
	Ydop       float64     `json:"ydop"`
	Vdop       float64     `json:"vdop"`
	Tdop       float64     `json:"tdop"`
	Hdop       float64     `json:"hdop"`
	Pdop       float64     `json:"pdop"`
	Gdop       float64     `json:"gdop"`
	Satellites []Satellite `json:"satellites"`
}

// GSTReport is pseudorange noise report
type GSTReport struct {
	Class  string    `json:"class"`
	Tag    string    `json:"tag"`
	Device string    `json:"device"`
	Time   time.Time `json:"time"`
	Rms    float64   `json:"rms"`
	Major  float64   `json:"major"`
	Minor  float64   `json:"minor"`
	Orient float64   `json:"orient"`
	Lat    float64   `json:"lat"`
	Lon    float64   `json:"lon"`
	Alt    float64   `json:"alt"`
}

// ATTReport reports vehicle-attitude from the digital compass or the gyroscope
type ATTReport struct {
	Class       string    `json:"class"`
	Tag         string    `json:"tag"`
	Device      string    `json:"device"`
	Time        time.Time `json:"time"`
	Heading     float64   `json:"heading"`
	MagSt       string    `json:"mag_st"`
	Pitch       float64   `json:"pitch"`
	PitchSt     string    `json:"pitch_st"`
	Yaw         float64   `json:"yaw"`
	YawSt       string    `json:"yaw_st"`
	Roll        float64   `json:"roll"`
	RollSt      string    `json:"roll_st"`
	Dip         float64   `json:"dip"`
	MagLen      float64   `json:"mag_len"`
	MagX        float64   `json:"mag_x"`
	MagY        float64   `json:"mag_y"`
	MagZ        float64   `json:"mag_z"`
	AccLen      float64   `json:"acc_len"`
	AccX        float64   `json:"acc_x"`
	AccY        float64   `json:"acc_y"`
	AccZ        float64   `json:"acc_z"`
	GyroX       float64   `json:"gyro_x"`
	GyroY       float64   `json:"gyro_y"`
	Depth       float64   `json:"depth"`
	Temperature float64   `json:"temperature"`
}

// VERSIONReport returns version details of gpsd client
type VERSIONReport struct {
	Class      string `json:"class"`
	Release    string `json:"release"`
	Rev        string `json:"rev"`
	ProtoMajor int    `json:"proto_major"`
	ProtoMinor int    `json:"proto_minor"`
	Remote     string `json:"remote"`
}

// DEVICESReport lists all devices connected to the system
type DEVICESReport struct {
	Class   string         `json:"class"`
	Devices []DEVICEReport `json:"devices"`
	Remote  string         `json:"remote"`
}

// DEVICEReport reports a state of a particular device
type DEVICEReport struct {
	Class     string  `json:"class"`
	Path      string  `json:"path"`
	Activated string  `json:"activated"`
	Flags     int     `json:"flags"`
	Driver    string  `json:"driver"`
	Subtype   string  `json:"subtype"`
	Bps       int     `json:"bps"`
	Parity    string  `json:"parity"`
	Stopbits  int     `json:"stopbits"`
	Native    int     `json:"native"`
	Cycle     float64 `json:"cycle"`
	Mincycle  float64 `json:"mincycle"`
}

// PPSReport is triggered on each pulse-per-second strobe from a device
type PPSReport struct {
	Class      string  `json:"class"`
	Device     string  `json:"device"`
	RealSec    float64 `json:"real_sec"`
	RealMusec  float64 `json:"real_musec"`
	ClockSec   float64 `json:"clock_sec"`
	ClockMusec float64 `json:"clock_musec"`
}

// TOFFReport is triggered on each PPS strobe from a device
type TOFFReport struct {
	Class     string  `json:"class"`
	Device    string  `json:"device"`
	RealSec   float64 `json:"real_sec"`
	RealNSec  float64 `json:"real_nsec"`
	ClockSec  float64 `json:"clock_sec"`
	ClockNSec float64 `json:"clock_nsec"`
}

// ERRORReport is an error response
type ERRORReport struct {
	Class   string `json:"class"`
	Message string `json:"message"`
}

// Satellite describes a location of a GPS satellite
type Satellite struct {
	PRN    int     `json:"PRN"`
	Az     float64 `json:"az"`
	El     float64 `json:"el"`
	Ss     float64 `json:"ss"`
	Used   bool    `json:"used"`
	GnssId int     `json:"gnssid"`
	SvId   int     `json:"svid"`
	Health float64 `json:"health"`
}

// GetUniqueId returns a unique satellite identifier
func (s *Satellite) GetUniqueId() string {
	return fmt.Sprintf("%d:%d", s.GnssId, s.SvId)
}

type GGAReport struct {
	Type          string  // Message type (GNGGA)
	Time          string  // UTC time in HHMMSS format
	Latitude      float64 // Decimal degrees
	LatitudeDir   string  // N or S
	Longitude     float64 // Decimal degrees
	LongitudeDir  string  // E or W
	Quality       int     // GPS Quality indicator
	NumSatellites int     // Number of satellites in use
	HDOP          float64 // Horizontal dilution of precision
	Altitude      float64 // Altitude above mean sea level
	AltitudeUnit  string  // Usually 'M' for meters
	Separation    float64 // Geoid separation
	SepUnit       string  // Usually 'M' for meters
	DiffAge       string  // Age of differential corrections
	DiffStation   string  // Differential base station ID
}

type RMCReport struct {
	Time             string  // UTC Time
	Status           string  // V = data invalid, A = data valid
	Latitude         float64 // Decimal degrees
	LatitudeDir      string  // N/S
	Longitude        float64 // Decimal degrees
	LongitudeDir     string  // E/W
	Speed            float64 // Speed over ground in knots
	CourseOverGround float64 // Course over ground in degrees
	Date             string  // Date in ddmmyy format
	MagneticVar      float64 // Magnetic variation
	MagneticVarDir   string  // Magnetic variation E/W
	PosMode          string  // Positioning mode indicator
	NavStatus        string  // Navigation status
}

func parseGGA(message string) (*GGAReport, error) {
	parts := strings.Split(message, ",")
	if len(parts) < 15 {
		return nil, fmt.Errorf("invalid GGA message format")
	}

	// Remove checksum from last field
	if idx := strings.Index(parts[14], "*"); idx != -1 {
		parts[14] = parts[14][:idx]
	}

	// Convert latitude from DDMM.MMMMM to decimal degrees
	lat, err := strconv.ParseFloat(parts[2], 64)
	if err != nil {
		return nil, fmt.Errorf("invalid latitude: %v", err)
	}
	latDeg := math.Floor(lat / 100)
	latMin := lat - (latDeg * 100)
	latitude := latDeg + latMin/60

	// Convert longitude from DDDMM.MMMMM to decimal degrees
	lon, err := strconv.ParseFloat(parts[4], 64)
	if err != nil {
		return nil, fmt.Errorf("invalid longitude: %v", err)
	}
	lonDeg := math.Floor(lon / 100)
	lonMin := lon - (lonDeg * 100)
	longitude := lonDeg + lonMin/60

	quality, _ := strconv.Atoi(parts[6])
	numSat, _ := strconv.Atoi(parts[7])
	hdop, _ := strconv.ParseFloat(parts[8], 64)
	alt, _ := strconv.ParseFloat(parts[9], 64)
	sep, _ := strconv.ParseFloat(parts[11], 64)

	// Convert latitude and apply sign based on direction
	if parts[3] == "S" {
		latitude = -latitude
	}

	// Convert longitude and apply sign based on direction
	if parts[5] == "W" {
		longitude = -longitude
	}

	return &GGAReport{
		Type:          parts[0],
		Time:          parts[1],
		Latitude:      latitude,
		LatitudeDir:   parts[3],
		Longitude:     longitude,
		LongitudeDir:  parts[5],
		Quality:       quality,
		NumSatellites: numSat,
		HDOP:          hdop,
		Altitude:      alt,
		AltitudeUnit:  parts[10],
		Separation:    sep,
		SepUnit:       parts[12],
		DiffAge:       parts[13],
		DiffStation:   parts[14],
	}, nil
}

func parseRMC(data string) (*RMCReport, error) {
	parts := strings.Split(data, ",")
	if len(parts) < 13 || !strings.HasPrefix(parts[0], "$GNRMC") {
		return nil, fmt.Errorf("invalid RMC message")
	}

	rmc := &RMCReport{
		Time:           parts[1],
		Status:         parts[2],
		LatitudeDir:    parts[4],
		LongitudeDir:   parts[6],
		Date:           parts[9],
		MagneticVarDir: parts[11],
		PosMode:        parts[12],
	}

	// Parse latitude
	if lat, err := strconv.ParseFloat(parts[3], 64); err == nil {
		degrees := math.Floor(lat / 100)
		minutes := lat - (degrees * 100)
		rmc.Latitude = degrees + minutes/60
		if rmc.LatitudeDir == "S" {
			rmc.Latitude = -rmc.Latitude
		}
	}

	// Parse longitude
	if lon, err := strconv.ParseFloat(parts[5], 64); err == nil {
		degrees := math.Floor(lon / 100)
		minutes := lon - (degrees * 100)
		rmc.Longitude = degrees + minutes/60
		if rmc.LongitudeDir == "W" {
			rmc.Longitude = -rmc.Longitude
		}
	}

	// Parse speed
	if parts[7] != "" {
		if speed, err := strconv.ParseFloat(parts[7], 64); err == nil {
			rmc.Speed = speed
		}
	}

	// Parse course over ground
	if parts[8] != "" {
		if cog, err := strconv.ParseFloat(parts[8], 64); err == nil {
			rmc.CourseOverGround = cog
		}
	}

	// Parse magnetic variation
	if parts[10] != "" {
		if magVar, err := strconv.ParseFloat(parts[10], 64); err == nil {
			rmc.MagneticVar = magVar
			if rmc.MagneticVarDir == "W" {
				rmc.MagneticVar = -rmc.MagneticVar
			}
		}
	}

	return rmc, nil
}

func GGAQualityToString(quality int) string {
	switch quality {
	case 0:
		return "No fix"
	case 1:
		return "autonomous GNSS fix"
	case 2:
		return "diﬀerential GNSS fix"
	case 4:
		return "RTK fixed"
	case 5:
		return "RTK float"
	case 6:
		return "estimated/dead reckoning fix"
	default:
		return "Unknown"
	}
}

func GnssIdToName(gnssId int) string {
	switch gnssId {
	case 0:
		return "GPS"
	case 1:
		return "SBAS"
	case 2:
		return "Galileo"
	case 3:
		return "BeiDou"
	case 4:
		return "IMES"
	case 5:
		return "QZSS"
	case 6:
		return "GLONASS"
	case 7:
		return "IRNSS"
	default:
		return "Unknown"
	}
}

// Dial opens a new connection to GPSD.
func Dial(address string) (*Session, error) {
	return dialCommon(net.Dial("tcp4", address))
}

// DialTimeout opens a new connection to GPSD with a timeout.
func DialTimeout(address string, to time.Duration) (*Session, error) {
	return dialCommon(net.DialTimeout("tcp4", address, to))
}

func dialCommon(c net.Conn, err error) (session *Session, e error) {
	session = new(Session)
	session.socket = c
	if err != nil {
		return nil, err
	}

	session.reader = bufio.NewReader(session.socket)
	session.reader.ReadString('\n')
	session.filters = make(map[string][]Filter)

	return
}

// Watch starts watching GPSD reports in a new goroutine.
//
// Example:
//
//	gps := gpsd.Dial(gpsd.DEFAULT_ADDRESS)
//	done := gpsd.Watch()
//	<- done
func (s *Session) Watch() (done chan bool) {
	fmt.Fprintf(s.socket, "?WATCH={\"enable\":true,\"json\":true,\"nmea\":true}")
	done = make(chan bool)

	go watch(done, s)

	return
}

// SendCommand sends a command to GPSD
func (s *Session) SendCommand(command string) {
	fmt.Fprintf(s.socket, "?"+command+";")
}

// AddFilter attaches a function which will be called for all
// GPSD reports with the given class. Callback functions have type Filter.
// An error is returned if the given class is invalid.
//
// Example:
//
//	gps := gpsd.Init(gpsd.DEFAULT_ADDRESS)
//	gps.AddFilter("TPV", func (r interface{}) {
//	  report := r.(*gpsd.TPVReport)
//	  fmt.Println(report.Time, report.Lat, report.Lon)
//	})
//	done := gps.Watch()
//	<- done
func (s *Session) AddFilter(class string, f Filter) error {

	// Check that class is a valid class string
	switch class {
	case "TPV", "SKY", "GST", "ATT", "VERSION", "DEVICES", "PPS", "TOFF", "ERROR", "GNGGA", "GNRMC":
	default:
		return errors.New("the provided class string is not a valid class")
	}

	s.filters[class] = append(s.filters[class], f)

	return nil
}

func (s *Session) deliverReport(class string, report interface{}) {
	for _, f := range s.filters[class] {
		f(report)
	}
}

// Close closes the connection to GPSD
func (s *Session) Close() error {
	if s.socket == nil {
		return errors.New("gpsd socket is already closed")
	}

	if err := s.socket.Close(); err != nil {
		return err
	}

	s.socket = nil
	return nil
}

func watch(done chan bool, s *Session) {
	// We're not using a JSON decoder because we first need to inspect
	// the JSON string to determine it's "class"
	for {
		if line, err := s.reader.ReadString('\n'); err == nil {
			var reportPeek gpsdReport
			if line[:6] == "$GNGGA" {
				if ggaReport, err2 := parseGGA(line); err2 == nil {
					s.deliverReport("GNGGA", ggaReport)
				} else {
					fmt.Println("GGA parsing error:", err)
				}
				continue
			}
			if line[:6] == "$GNRMC" {
				if rmcReport, err2 := parseRMC(line); err2 == nil {
					s.deliverReport("GNRMC", rmcReport)
				} else {
					fmt.Println("RMC parsing error:", err)
				}
				continue
			}
			if line[:1] != "{" {
				continue
			}
			lineBytes := []byte(line)
			if err = json.Unmarshal(lineBytes, &reportPeek); err == nil {
				if len(s.filters[reportPeek.Class]) == 0 {
					continue
				}

				if report, err2 := unmarshalReport(reportPeek.Class, lineBytes); err2 == nil {
					s.deliverReport(reportPeek.Class, report)
				} else {
					fmt.Println("JSON parsing error 2:", err)
				}
			} else {
				fmt.Println("JSON parsing error:", err)
			}
		} else {
			if !errors.Is(err, net.ErrClosed) {
				fmt.Println("Stream reader error (is gpsd running?):", err)
			}
			if errors.Is(err, io.EOF) || errors.Is(err, net.ErrClosed) {
				break
			}
		}
	}
	done <- true
}

func unmarshalReport(class string, bytes []byte) (interface{}, error) {
	var err error

	switch class {
	case "TPV":
		var r *TPVReport
		err = json.Unmarshal(bytes, &r)
		return r, err
	case "SKY":
		var r *SKYReport
		err = json.Unmarshal(bytes, &r)
		return r, err
	case "GST":
		var r *GSTReport
		err = json.Unmarshal(bytes, &r)
		return r, err
	case "ATT":
		var r *ATTReport
		err = json.Unmarshal(bytes, &r)
		return r, err
	case "VERSION":
		var r *VERSIONReport
		err = json.Unmarshal(bytes, &r)
		return r, err
	case "DEVICE":
		var r *DEVICEReport
		err = json.Unmarshal(bytes, &r)
		return r, err
	case "DEVICES":
		var r *DEVICESReport
		err = json.Unmarshal(bytes, &r)
		return r, err
	case "PPS":
		var r *PPSReport
		err = json.Unmarshal(bytes, &r)
		return r, err
	case "TOFF":
		var r *TOFFReport
		err = json.Unmarshal(bytes, &r)
		return r, err
	case "ERROR":
		var r *ERRORReport
		err = json.Unmarshal(bytes, &r)
		return r, err
	}

	return nil, err
}
