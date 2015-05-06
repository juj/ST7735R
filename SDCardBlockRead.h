// A low level library for accessing the SD card in a fast way.
// Written by Jukka Jylänki.
// This code is released to public domain.
#pragma once

// Configure these:
#define SDCARD_CS_PIN 4
#define SDCARD_CS_PORT PORTD
#define SDCARD_CS_BIT (1 << PD4)

#define BEGIN_SDCARD() (SDCARD_CS_PORT &= ~SDCARD_CS_BIT)
#define END_SDCARD() (SDCARD_CS_PORT |= SDCARD_CS_BIT)

uint16_t SdCard_numBytesRead = 513;

// send command and return error code.  Return zero for OK
void SdCard_cardCommand(uint8_t cmd, uint32_t arg)
{
  // send command
  WRITE_SPI_SYNC(cmd | 0x40);

  // send argument
  WRITE_SPI_SYNC((uint8_t)(arg >> 24));
  WRITE_SPI_SYNC((uint8_t)(arg >> 16));
  WRITE_SPI_SYNC((uint8_t)(arg >> 8));
  WRITE_SPI_SYNC((uint8_t)arg);

  WRITE_SPI_SYNC(0xFF); // Should be CRC, but send a dummy data, not computed.

  // wait for response
  while((SPDR & 0x80) != 0) WRITE_SPI_SYNC(0xFF);
}

void SdCard_BeginRead(uint32_t block)
{
  // SdCard_numBytesRead should be 513 at this point to indicate no read is going on!

  SdCard_cardCommand(CMD17, block);

  // Each SD card block read transfer is 512 bytes + 1 byte for CRC at the end.
  // numBytesRead tracks how many byte transfers we have pumped through SPI so far.
  SdCard_numBytesRead = 0;

  // wait for block read start block token
  WRITE_SPI_SYNC(0xFF);
  while(SPDR == 0xFF) WRITE_SPI_SYNC(0xFF);
}

void __attribute__((always_inline)) SdCard_EndRead()
{
  while(SdCard_numBytesRead++ < 513) WRITE_SPI_SYNC(0xFF);
}

void SdCard_readData(uint32_t block, uint16_t offset, uint16_t count, uint8_t* dst)
{
  SdCard_BeginRead(block);

  // start first spi transfer
  WRITE_SPI_NOWAIT(0xFF);

  // Seek the block read to the position that we want to read from.
  for(;SdCard_numBytesRead < offset; SdCard_numBytesRead++)
  {
    NOP;
    WAIT_SPI;
    WRITE_SPI_NOWAIT(0xFF);
  }
  // Transfer the interesting bytes.
  const uint16_t n = count - 1;
  for(uint16_t i = 0; i < n; ++i)
  {
    NOP; // Wins 12 msecs (522msec -> 510 msec, tested when reading a whole block).
    WAIT_SPI;
    dst[i] = SPDR;
    WRITE_SPI_NOWAIT(0xFF);
  }
  // And the last byte.
  NOP;
  WAIT_SPI;
  dst[n] = SPDR;

  SdCard_numBytesRead += count;

  // We must pump the block read through to the end of the block, so read the bytes
  // and discard the results.
  SdCard_EndRead();
}

void __attribute__((always_inline)) SdCard_SkipBytes(int numBytes)
{
  //assert(SdCard_numBytesRead < 512);
  SdCard_numBytesRead += numBytes;
  //assert(SdCard_numBytesRead <= 512);
  while(numBytes-- > 0) WRITE_SPI_SYNC(0xFF);
}

uint8_t __attribute__((always_inline)) SdCard_readU8()
{
  WRITE_SPI_SYNC(0xFF);
  ++SdCard_numBytesRead;
  return SPDR;
}

uint16_t SdCard_readU16LSB()
{
  WRITE_SPI_SYNC(0xFF);
  uint8_t lo = SPDR;
  WRITE_SPI_SYNC(0xFF);
  uint8_t hi = SPDR;
  SdCard_numBytesRead += 2;
  return ((uint16_t)hi << 8) | lo;
}

uint32_t SdCard_readU32LSB()
{
  WRITE_SPI_SYNC(0xFF);
  uint8_t b1 = SPDR;
  WRITE_SPI_SYNC(0xFF);
  uint8_t b2 = SPDR;
  WRITE_SPI_SYNC(0xFF);
  uint8_t b3 = SPDR;
  WRITE_SPI_SYNC(0xFF);
  uint8_t b4 = SPDR;
  SdCard_numBytesRead += 4;
  return (((uint32_t)(((uint16_t)b4 << 8) | b3)) << 16) | ((uint32_t)(((uint16_t)b2 << 8) | b1));
}

uint32_t SDCard_GetFileStartingBlock(const char *filename)
{
  BEGIN_SDCARD();
  File f = SD.open(filename, FILE_READ);
  if (!f)
  {
    Serial.print(F("Failed to open file "));
    Serial.println(filename);
    END_SDCARD();
    return (uint32_t)-1;
  }
  uint32_t bgnBlock, endBlock;
  uint8_t isContiguous = f._file->contiguousRange(&bgnBlock, &endBlock);
  if (!isContiguous) Serial.println(F("!isContiguous"));
  f.close();
  END_SDCARD();
  return bgnBlock;
}

