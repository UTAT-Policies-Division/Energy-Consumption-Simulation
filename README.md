# Drone Integrated Last-Mile Delivery Simulation

## Description

This repository holds the code for simulating truck-drone last-mile delivery system.

1. Road network via OSMNx, utilizes GIS tools for cleaning and removing NFZs.
3. BET with Coleman Inflow Model Propeller Energy Consumption Model for drones, implemented with arithmetic optimizations.
4. Realistic environmental conditions such as wind vector fields, temperature scalar fields, traffic drift velocity fields etc.
2. Edge calibration uses active multiprocessing, with design decisions for efficiency such as quantization of payload weights for drone. 
6. ACO System for one-truck-one-drone HDP, utilizes custom multiprocessing primitives and a custom solution communication system.

## Contributors

Parth Singh

Tafia Mehbuba Islam

Kevin Caldwell

Sofiya P'yavka
