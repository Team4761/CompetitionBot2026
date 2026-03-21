# Perfect Peanut

## Subsystems

### Swerve
- 4x dual kraken modules from Swerve Drive Specialties

### Vision
- 2x Orange Pi coprocessors
- 4x arducams

### Intake
- 1x kraken for rollers
- 1x kraken for deploy

### Shooter
- 1-2x neo for rollers
- 1x neo for kicker/uppinator
- 1x kraken for spitter

## Software

### Vision + pose pipeline
- Photonvision for polling april tags from arducams
- CTRE's swervedrive's pose update function

### Autos
- Pathplanner