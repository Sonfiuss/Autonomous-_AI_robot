# Omni Wheel Robot 2D Simulator

Web-based 2D simulator for a 3-wheel omni-directional robot. Robot physics run in Python; rendering is HTML/CSS/Canvas.

## Run

```bash
pip install -r requirements.txt
python main.py
# Open http://localhost:5000 in your browser
```

Controls:
- W/A/D: accelerate wheels A/B/C forward
- Up/Left/Down arrows: accelerate wheels A/B/C reverse
- R: reset simulation

## Tests

```bash
pytest
```
