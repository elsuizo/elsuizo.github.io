+++
title = "Primeros pasos con Rust"
date = 2019-04-18
[taxonomies]
categories = ["article"]
tags = ["piola"]
+++


### Primeros pasos con Rust y como no morir en el intento

Rust es un lenguaje de programacion particular que viene a solucionar un problema
que existe desde hace mucho tiempo que es el manejo de memoria seguro y eficiente

```rust
 pub fn screw_lerp(begin: &Self, end: &Self, tau: T) -> Self {
     let one = T::one();
     let mut start = begin.clone();
     // TODO(elsuizo:2021-05-23): this is from the python implementation that refers to a paper
     // that "ensure we always find closest solution, See Kavan and Zara 2005"
     if (start.real() * end.real()).real() < T::zero() {
         start.q_real = begin.real() * -one;
     }
     start * (start.inverse() * *end).pow(tau)
 }
```
