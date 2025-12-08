# IEEE Conference Paper - TM5-900 Vlogger System

This directory contains the IEEE conference paper for the Autonomous Vlogging System project.

## Paper Contents

The paper includes:
- **Abstract**: Overview of the autonomous vlogging system
- **Introduction**: Problem statement and key contributions
- **Related Work**: Visual servoing, face detection, safety, and gesture recognition
- **Methodology**: Detailed technical implementation including:
  - System architecture (camera, detection, control, robot layers)
  - Coordinate transformation equations
  - Distance estimation using monocular vision
  - Safety mechanisms
- **Experimental Setup**: Hardware, software stack, and startup procedure
- **Results**: Performance metrics including 100x improvement over baseline
- **Conclusions**: Achievements and future work directions

## Compilation Instructions

### Option 1: Install LaTeX (Recommended)

```bash
# Install TeX Live (full LaTeX distribution)
sudo apt-get update
sudo apt-get install texlive-latex-base texlive-latex-extra

# Compile the paper
cd /home/robot/workspace2/team11_ws_final_project/Robotics_Final_Project/ieeeconf
pdflatex root.tex
pdflatex root.tex  # Run twice to resolve references

# View the PDF
evince root.pdf  # or any PDF viewer
```

### Option 2: Use Online LaTeX Editor

1. Go to [Overleaf](https://www.overleaf.com/)
2. Create a new blank project
3. Upload the following files:
   - `root.tex`
   - `ieeeconf.cls`
4. Click "Recompile" to generate the PDF
5. Download the compiled PDF

### Option 3: Install Minimal LaTeX

```bash
# Install minimal LaTeX setup (faster installation)
sudo apt-get install texlive-latex-base

# Compile
cd /home/robot/workspace2/team11_ws_final_project/Robotics_Final_Project/ieeeconf
pdflatex root.tex
pdflatex root.tex
```

## Files in This Directory

- **root.tex**: Main LaTeX source file containing the complete paper
- **ieeeconf.cls**: IEEE conference document class file
- **root.pdf**: Compiled PDF (generated after compilation)
- **README.md**: This file

## Paper Statistics

- **Pages**: Approximately 7-8 pages in two-column IEEE format
- **Sections**: 7 main sections + references
- **Equations**: 15 mathematical equations
- **References**: 10 citations covering visual servoing, face detection, safety, and robotics

## Key Technical Details Covered

1. **Real-time Performance**
   - 30 FPS face tracking
   - Sub-100ms end-to-end latency
   - 100x improvement over baseline (0.3 FPS → 30 FPS)

2. **Detection Algorithms**
   - MediaPipe Face Mesh (468 landmarks)
   - MediaPipe Hands (21 landmarks)
   - Face size calculation and center point extraction

3. **Control System**
   - Image-to-robot coordinate transformation
   - Monocular distance estimation using face size
   - Safety mechanisms (3-layer protection)
   - Rate limiting and workspace constraints

4. **Hardware Integration**
   - TM5-900 robot arm specifications
   - USB webcam configuration (640×480 @ 30 FPS)
   - ROS2 Jazzy middleware
   - Camera parameter optimization (brightness, gain, exposure)

5. **Safety Features**
   - Primary safety check (no face = no movement)
   - Position data clearing and validation
   - Workspace limits (X: 100-600mm, Y: 0-600mm, Z: 200-700mm)
   - Rate limiting (max 1 Hz movement)

## Customization

To modify the paper:

1. Edit `root.tex` using any text editor
2. Key sections to customize:
   - **Title**: Line 19-21
   - **Authors**: Line 24-26
   - **Abstract**: Lines 39-42
   - **Content**: Lines 47-363
   - **References**: Lines 374-396

3. Recompile after changes:
   ```bash
   pdflatex root.tex
   pdflatex root.tex  # Second run resolves cross-references
   ```

## Troubleshooting

**Missing packages during compilation:**
```bash
sudo apt-get install texlive-latex-extra texlive-fonts-recommended
```

**Bibliography not appearing:**
```bash
# Run pdflatex twice
pdflatex root.tex
pdflatex root.tex
```

**Errors with special characters:**
- Use LaTeX escapes: `\_` for underscore, `\&` for ampersand
- Math mode for symbols: `$\times$` for multiplication

**PDF not opening:**
```bash
# Check if PDF was generated
ls -lh root.pdf

# Install PDF viewer if needed
sudo apt-get install evince  # or okular, xpdf, etc.
```

## Citation

If you reference this work, please cite:

```
Team 11, "Autonomous Vlogging System with Real-Time Face Tracking
Using TM5-900 Robot Arm," Robotics Final Project, 2025.
```

## Contact

For questions about the paper content or compilation:
- Review the project README.md in the parent directory
- Check the source code in `src/vlogger_system/vlogger_system/vlogger_control.py`
- See technical documentation in the project root

---

**Last Updated**: December 8, 2025
**Status**: Ready for compilation and submission
