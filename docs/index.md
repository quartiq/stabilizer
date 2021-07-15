---
title: Home
layout: default
nav_order: 1
permalink: /
---

# Stabilizer
{: .no_toc }

## Table of Contents
{: .no_toc .text-delta }

1. TOC
{:toc}

## Overview

Stabilizer is a flexible tool designed for quantum physics experiments. Fundamentally, Stabilizer
samples up two two analog input signals, performs digital signal processing internally, and then
generates up to two output signals.

Stabilizer firmware supports run-time configuration of the internal signal processing algorithms,
which allows for a wide variety of experimental uses, such as digital filter design or
implementation of digital lockin schemes.

This documentation is intended to bring a user up to speed on using Stabilizer and the firmware
provided by Quartiq.

## Hardware

The Stabilizer hardware is managed via a [separate repository](https://github.com/sinara-hw/Stabilizer).

[![Hardware](https://github.com/sinara-hw/Stabilizer/wiki/Stabilizer_v1.0_top_small.jpg)](https://github.com/sinara-hw/Stabilizer)

## Applications

This firmware offers a library of hardware and software functionality targeting the use of the Stabilizer hardware in various digital signal processing applications commonly occurring in Quantum Technology.
It provides abstractions over the fast analog inputs and outputs, time stamping, Pounder DDS interfaces and a collection of tailored and optimized digital signal processing algorithms (IIR, FIR, Lockin, PLL, reciprocal PLL, Unwrapper, Lowpass, Cosine-Sine, Atan2).
An application, which is the compiled firmware running on the device, can compose and configure these hardware and software components to implement different use cases.
Several applications are provided by default.

The following documentation links contain the application-specific settings and telemetry
information.

| Application | Documentation | Application Description |
| :---: | :--: | :---- |
| `dual-iir` | [Link]({{site.baseurl}}/firmware/dual_iir/index.html) | Two channel biquad IIR filter
| `lockin` | [Link]({{site.baseurl}}/firmware/lockin/index.html) | Lockin amplifier support various various reference sources |

### Library Documentation
The Stabilizer library docs contain documentation for common components used in all Stabilizer
applications.

The Stabilizer library documentation is available [here]({{site.baseurl}}/firmware/stabilizer/index.html).
