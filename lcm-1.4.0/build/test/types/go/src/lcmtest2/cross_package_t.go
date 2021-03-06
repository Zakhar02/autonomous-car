// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

package lcmtest2

import (
	"encoding/binary"
	"fmt"
	"math"
	"math/bits"
	"lcmtest"
)

const _ = math.Pi
const _ = bits.UintSize

const cross_package_t_Fingerprint uint64 = 0xbbd3dd8a23ec1955

type Lcmtest2CrossPackageT struct {
	Primitives lcmtest.LcmtestPrimitivesT `primitives`
	Another    Lcmtest2AnotherTypeT       `another`
}

// Copy creates a deep copy
// TODO: fix the fugly x and p names...
func (x *Lcmtest2CrossPackageT) Copy() (p Lcmtest2CrossPackageT) {
	p.Primitives = x.Primitives.Copy()

	p.Another = x.Another.Copy()

	return
}

// Encode encodes a message (fingerprint & data) into binary form
//
// returns Encoded data or error
func (p *Lcmtest2CrossPackageT) Encode() (data []byte, err error) {
	var size int
	if size, err = p.Size(); err != nil {
		return
	}

	data = make([]byte, 8+size)
	binary.BigEndian.PutUint64(data, Lcmtest2CrossPackageT_Fingerprint())

	var d []byte
	if d, err = p.MarshalBinary(); err != nil {
		return
	}

	if copied := copy(data[8:], d); copied != size {
		return []byte{},
			fmt.Errorf("Encoding error, buffer not filled (%v != %v)", copied, size)
	}
	return
}

// MarshalBinary implements the BinaryMarshaller interface
func (p *Lcmtest2CrossPackageT) MarshalBinary() (data []byte, err error) {
	var size int
	if size, err = p.Size(); err != nil {
		return
	}

	data = make([]byte, size)
	offset := 0

	// LCM struct name: primitives
	{
		var tmp []byte
		if tmp, err = p.Primitives.MarshalBinary(); err != nil {
			return
		}
		offset += copy(data[offset:], tmp)
	}

	// LCM struct name: another
	{
		var tmp []byte
		if tmp, err = p.Another.MarshalBinary(); err != nil {
			return
		}
		offset += copy(data[offset:], tmp)
	}

	return
}

// Decode decodes a message (fingerprint & data) from binary form
// and verifies that the fingerprint match the expected
//
// param data The buffer containing the encoded message
// returns Error
func (p *Lcmtest2CrossPackageT) Decode(data []byte) (err error) {
	length := len(data)
	if length < 8 {
		return fmt.Errorf("Missing fingerprint in buffer")
	}

	if fp := binary.BigEndian.Uint64(data[:8]); fp != Lcmtest2CrossPackageT_Fingerprint() {
		return fmt.Errorf("Fingerprints does not match (got %x expected %x)",
			fp, Lcmtest2CrossPackageT_Fingerprint())
	}

	if err = p.UnmarshalBinary(data[8:]); err != nil {
		return
	}

	length -= 8
	var size int
	if size, err = p.Size(); err != nil {
		return
	}
	if length != size {
		return fmt.Errorf("Missing data in buffer (size missmatch, got %v expected %v)",
			length, size)
	}

	return
}

// UnmarshalBinary implements the BinaryUnmarshaler interface
func (p *Lcmtest2CrossPackageT) UnmarshalBinary(data []byte) (err error) {
	offset := 0

	if err = p.Primitives.UnmarshalBinary(data[offset:]); err != nil {
		return
	}
	{
		var size int
		if size, err = p.Primitives.Size(); err != nil {
			return
		}
		offset += size
	}

	if err = p.Another.UnmarshalBinary(data[offset:]); err != nil {
		return
	}
	{
		var size int
		if size, err = p.Another.Size(); err != nil {
			return
		}
		offset += size
	}

	return
}

// Fingerprint generates the LCM fingerprint value for this message
func Lcmtest2CrossPackageT_Fingerprint(path ...uint64) uint64 {
	for _, v := range path {
		if v == cross_package_t_Fingerprint {
			return 0
		}
	}

	path = append(path, cross_package_t_Fingerprint)
	return bits.RotateLeft64(cross_package_t_Fingerprint+
		lcmtest.LcmtestPrimitivesT_Fingerprint(path...)+
		Lcmtest2AnotherTypeT_Fingerprint(path...), 1)
}

// Size returns the size of this message in bytes
func (p *Lcmtest2CrossPackageT) Size() (size int, err error) {

	{
		var tmp int
		if tmp, err = p.Primitives.Size(); err != nil {
			return
		}
		size += tmp
	}

	{
		var tmp int
		if tmp, err = p.Another.Size(); err != nil {
			return
		}
		size += tmp
	}

	return
}
