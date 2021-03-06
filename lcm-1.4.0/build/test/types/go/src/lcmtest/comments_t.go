// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

package lcmtest

import (
	"encoding/binary"
	"fmt"
	"math"
	"math/bits"
)

const _ = math.Pi
const _ = bits.UintSize

const comments_t_Fingerprint uint64 = 0x4c8ffa86357c7d7e

// LCM constants
const (
	LcmtestCommentsT_ConstField int8 = 5
)

// Contains a number of primitive data types
type LcmtestCommentsT struct {
	// Comments immediately preceding a field are attached to that field.
	fieldA int8    `field_a`
	// Both single line comments.
	//  And multi-line
	// comments.
	// 
	FieldB int16   `field_b`
	// Test empty comments.
	// 
	FieldC int32   `field_c`
	// 
	// 
	FieldD int64   `field_d`
	// 
	// 
	FieldE float32 `field_e`
	Array  []int32 `array`
}

// Copy creates a deep copy
// TODO: fix the fugly x and p names...
func (x *LcmtestCommentsT) Copy() (p LcmtestCommentsT) {
	p.fieldA = x.fieldA

	p.FieldB = x.FieldB

	p.FieldC = x.FieldC

	p.FieldD = x.FieldD

	p.FieldE = x.FieldE

	p.Array = make([]int32, p.fieldA)
	for i0 := int8(0); i0 < p.fieldA; i0++ {
		p.Array[i0] = x.Array[i0]
	}

	return
}

// Encode encodes a message (fingerprint & data) into binary form
//
// returns Encoded data or error
func (p *LcmtestCommentsT) Encode() (data []byte, err error) {
	var size int
	if size, err = p.Size(); err != nil {
		return
	}

	data = make([]byte, 8+size)
	binary.BigEndian.PutUint64(data, LcmtestCommentsT_Fingerprint())

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
func (p *LcmtestCommentsT) MarshalBinary() (data []byte, err error) {
	var size int
	if size, err = p.Size(); err != nil {
		return
	}

	data = make([]byte, size)
	offset := 0

	// LCM struct name: field_a
	data[offset] = byte(p.fieldA)
	offset += 1

	// LCM struct name: field_b
	binary.BigEndian.PutUint16(data[offset:],
		uint16(p.FieldB))
	offset += 2

	// LCM struct name: field_c
	binary.BigEndian.PutUint32(data[offset:],
		uint32(p.FieldC))
	offset += 4

	// LCM struct name: field_d
	binary.BigEndian.PutUint64(data[offset:],
		uint64(p.FieldD))
	offset += 8

	// LCM struct name: field_e
	binary.BigEndian.PutUint32(data[offset:],
		math.Float32bits(p.FieldE))
	offset += 4

	// LCM struct name: array
	for i0 := int8(0); i0 < p.fieldA; i0++ {
		binary.BigEndian.PutUint32(data[offset:],
			uint32(p.Array[i0]))
		offset += 4
	}

	return
}

// Decode decodes a message (fingerprint & data) from binary form
// and verifies that the fingerprint match the expected
//
// param data The buffer containing the encoded message
// returns Error
func (p *LcmtestCommentsT) Decode(data []byte) (err error) {
	length := len(data)
	if length < 8 {
		return fmt.Errorf("Missing fingerprint in buffer")
	}

	if fp := binary.BigEndian.Uint64(data[:8]); fp != LcmtestCommentsT_Fingerprint() {
		return fmt.Errorf("Fingerprints does not match (got %x expected %x)",
			fp, LcmtestCommentsT_Fingerprint())
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
func (p *LcmtestCommentsT) UnmarshalBinary(data []byte) (err error) {
	offset := 0

	p.fieldA = int8(data[offset])
	offset += 1

	p.FieldB = int16(binary.BigEndian.Uint16(data[offset:]))
	offset += 2

	p.FieldC = int32(binary.BigEndian.Uint32(data[offset:]))
	offset += 4

	p.FieldD = int64(binary.BigEndian.Uint64(data[offset:]))
	offset += 8

	p.FieldE = math.Float32frombits(binary.BigEndian.Uint32(data[offset:]))
	offset += 4

	p.Array = make([]int32, p.fieldA)
	for i0 := int8(0); i0 < p.fieldA; i0++ {
		p.Array[i0] = int32(binary.BigEndian.Uint32(data[offset:]))
		offset += 4
	}

	return
}

// FieldA() returns the value of dynamic array size attribute
// LcmtestCommentsT.field_a.
// And validates that the size is correct for all fields in which it is used.
func (p *LcmtestCommentsT) FieldA() (int8, error) {
	// Set value to first dynamic array using this size
	// Array
	p.fieldA = int8(len(p.Array))

	// Return size
	return p.fieldA, nil
}

// Fingerprint generates the LCM fingerprint value for this message
func LcmtestCommentsT_Fingerprint(path ...uint64) uint64 {
	for _, v := range path {
		if v == comments_t_Fingerprint {
			return 0
		}
	}

	path = append(path, comments_t_Fingerprint)
	return bits.RotateLeft64(comments_t_Fingerprint, 1)
}

// Size returns the size of this message in bytes
func (p *LcmtestCommentsT) Size() (size int, err error) {

	// Validate and populate p.fieldA
	if _, err = p.FieldA(); err != nil {
		return
	}
	size += 1 // p.fieldA

	size += 2 // p.FieldB

	size += 4 // p.FieldC

	size += 8 // p.FieldD

	size += 4 // p.FieldE

	for i0 := int8(0); i0 < p.fieldA; i0++ {
		size += 4 // p.Array
	}

	return
}
