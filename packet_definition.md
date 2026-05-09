# SPI Packet Definition

## 8-Byte Frame Table

| Byte Index | Constant Name | Size | Description |
| :--- | :--- | :--- | :--- |
| 0 | `SPI_IDX_HEADER` | 1 byte | Sync byte (`0xA5`) |
| 1 | `SPI_IDX_STATE` | 1 byte | Current FSM State |
| 2 | `SPI_IDX_FLOOR` | 1 byte | Current physical floor (1-4) |
| 3 | `SPI_IDX_DIRECTION`| 1 byte | Current movement direction |
| 4 | `SPI_IDX_QUEUE_HI` | 1 byte | Floor requests bitmask (floors 5-8, unused) |
| 5 | `SPI_IDX_QUEUE_LO` | 1 byte | Floor requests bitmask (floors 1-4) |
| 6 | `SPI_IDX_FLAGS` | 1 byte | Master-to-Slave command flags |
| 7 | `SPI_IDX_CHECKSUM` | 1 byte | XOR checksum of bytes 0-6 |