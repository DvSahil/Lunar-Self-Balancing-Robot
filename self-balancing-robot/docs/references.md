# References

Look these up directly and cite what you actually use.

* University of Michigan **Control Tutorials for MATLAB and Simulink (CTMS)**, "Inverted Pendulum: System Modeling" and
  "Inverted Pendulum: PID / State-Space Controller Design" (ctms.engin.umich.edu). The cart-pole model and the
  linearisation used in doc 1.
* K. J. Åström and R. M. Murray, **Feedback Systems: An Introduction for Scientists and Engineers**
  (Princeton University Press; free PDF from the authors). Chapters on PID control and state feedback, with the
  inverted pendulum as an example.
* S. Colton, **"The Balance Filter"** (MIT, 2007). A short, readable explanation of the complementary filter for
  balancing robots.
* R. Mahony, T. Hamel, J.-M. Pflimlin, **"Nonlinear Complementary Filters on the Special Orthogonal Group"**,
  IEEE Transactions on Automatic Control, 2008. The theory behind the more general attitude filters.
* InvenSense **MPU-6000 / MPU-6050 Product Specification** and **Register Map and Descriptions**. The register
  addresses and sensitivities used in `imu.h`.
* Toshiba **TB6612FNG Datasheet**. The truth table of IN1/IN2/STBY/PWM used in `motors.h`.
* Arduino language reference (`attachInterrupt`, `analogWrite`, `Wire`, `EEPROM`) on arduino.cc.
