# LL – Low-Level Motor & Encoder Interface

The **LL layer** runs on the Arduino and acts as a **pure hardware interface**.
It applies motor commands and reports encoder data. **No control logic is implemented here.**

---

## Responsibilities

* Receive PWM duty cycle commands from L1 over UART
* Drive motor direction and PWM outputs
* Read quadrature encoders
* Send **raw 16-bit encoder counts** back to L1

---

## Explicitly NOT in LL

* No PID or control loops
* No kinematics
* No velocity regulation
* No decision logic

All control is handled in **L1**.

---

## UART Interface

### L1 → LL (Motor command frame)

```
[START][TL][TR][BL][BR][CHECKSUM]
```

* Signed values encode direction and duty cycle

### LL → L1 (Encoder frame)

```
[START][TL_lo][TL_hi][TR_lo][TR_hi][BL_lo][BL_hi][BR_lo][BR_hi][CHECKSUM]
```

* 16-bit encoder counts
* Wrap-around handled in L1

---

## Timing

* LL runs as fast as possible
* No blocking delays
* Control timing is dictated by L1

---

## Notes

LL is intentionally minimal. If behavior is incorrect, the issue is almost certainly in **L1**, not LL.
