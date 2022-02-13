# Digital PID Controller for laboratory usage (Arduino based)

### Control Systems Seminar(I) Project 

<b>Project Title  :</b> Designing a PID controller having analog input and analog output, using digital and analog components for laboratory use

<b>Project Report :</b> [Link](https://github.com/pranabendra/digital-pid-controller/blob/master/Report/Control_Seminar_Project_PPC.pdf)

<b>Project Video  :</b> https://youtu.be/duWAQ8FL80s

---

### Published Paper

<b>Paper Title  :</b> A Framework for Digital PID Controller for Undergraduate Control Systems Laboratory Usage

<b>Paper Link   :</b> [DOI: 10.1109/INDICON52576.2021.9691674](https://doi.org/10.1109/INDICON52576.2021.9691674)

---

### Future work

Improvements can definitely be made to this framework, such as - 
1. A microcontroller having an in-built DAC (such as Arduino Due) can be used. This will eliminate the passive RC filter bank. But, we have to keep in mind that the current system is 5V tolerant. If we use a 3.3V tolerant microcontroller, the system needs a little re-design.
2. One SMPS with an inverting buck boost converter can be used instead of two SMPS. This is just to match the two power rails as close as possible in terms of DC level and ripple factor.
3. Anti-integral windup.
4. Anti-derivative kick. 

---

### Citation

```
@inproceedings{chandra2021framework,
  title={A Framework for Digital PID Controller for Undergraduate Control Systems Laboratory Usage},
  author={Chandra, Pranabendra Prasad},
  booktitle={2021 IEEE 18th India Council International Conference (INDICON)},
  pages={1--7},
  year={2021},
  organization={IEEE}
}
```
