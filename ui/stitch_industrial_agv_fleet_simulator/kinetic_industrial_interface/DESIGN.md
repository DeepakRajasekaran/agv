---
name: Kinetic Industrial Interface
colors:
  surface: '#0b1326'
  surface-dim: '#0b1326'
  surface-bright: '#31394d'
  surface-container-lowest: '#060e20'
  surface-container-low: '#131b2e'
  surface-container: '#171f33'
  surface-container-high: '#222a3d'
  surface-container-highest: '#2d3449'
  on-surface: '#dae2fd'
  on-surface-variant: '#d1c6ab'
  inverse-surface: '#dae2fd'
  inverse-on-surface: '#283044'
  outline: '#9a9078'
  outline-variant: '#4d4632'
  surface-tint: '#eec200'
  primary: '#ffecb9'
  on-primary: '#3c2f00'
  primary-container: '#facc15'
  on-primary-container: '#6c5700'
  inverse-primary: '#735c00'
  secondary: '#4edea3'
  on-secondary: '#003824'
  secondary-container: '#00a572'
  on-secondary-container: '#00311f'
  tertiary: '#ffe8e6'
  on-tertiary: '#68000a'
  tertiary-container: '#ffc2bd'
  on-tertiary-container: '#b2131f'
  error: '#ffb4ab'
  on-error: '#690005'
  error-container: '#93000a'
  on-error-container: '#ffdad6'
  primary-fixed: '#ffe083'
  primary-fixed-dim: '#eec200'
  on-primary-fixed: '#231b00'
  on-primary-fixed-variant: '#574500'
  secondary-fixed: '#6ffbbe'
  secondary-fixed-dim: '#4edea3'
  on-secondary-fixed: '#002113'
  on-secondary-fixed-variant: '#005236'
  tertiary-fixed: '#ffdad7'
  tertiary-fixed-dim: '#ffb3ad'
  on-tertiary-fixed: '#410004'
  on-tertiary-fixed-variant: '#930013'
  background: '#0b1326'
  on-background: '#dae2fd'
  surface-variant: '#2d3449'
typography:
  headline-lg:
    fontFamily: JetBrains Mono
    fontSize: 32px
    fontWeight: '700'
    lineHeight: 40px
    letterSpacing: -0.02em
  headline-md:
    fontFamily: JetBrains Mono
    fontSize: 24px
    fontWeight: '600'
    lineHeight: 32px
    letterSpacing: -0.01em
  headline-sm:
    fontFamily: JetBrains Mono
    fontSize: 20px
    fontWeight: '600'
    lineHeight: 28px
    letterSpacing: '0'
  body-lg:
    fontFamily: Inter
    fontSize: 16px
    fontWeight: '400'
    lineHeight: 24px
    letterSpacing: '0'
  body-md:
    fontFamily: Inter
    fontSize: 14px
    fontWeight: '400'
    lineHeight: 20px
    letterSpacing: '0'
  data-mono:
    fontFamily: JetBrains Mono
    fontSize: 14px
    fontWeight: '500'
    lineHeight: 20px
    letterSpacing: 0.02em
  label-caps:
    fontFamily: JetBrains Mono
    fontSize: 12px
    fontWeight: '700'
    lineHeight: 16px
    letterSpacing: 0.05em
  label-sm:
    fontFamily: Inter
    fontSize: 11px
    fontWeight: '600'
    lineHeight: 14px
    letterSpacing: 0.01em
rounded:
  sm: 0.125rem
  DEFAULT: 0.25rem
  md: 0.375rem
  lg: 0.5rem
  xl: 0.75rem
  full: 9999px
spacing:
  base: 4px
  xs: 4px
  sm: 8px
  md: 16px
  lg: 24px
  xl: 32px
  gutter: 16px
  panel-padding: 20px
---

## Brand & Style

This design system is engineered for high-stakes industrial automation and AGV (Automated Guided Vehicle) fleet management. The personality is uncompromisingly functional, precise, and urgent. It prioritizes rapid information retrieval and error-free operation in low-light warehouse environments.

The aesthetic blends **Modern Industrialism** with **Functional Minimalism**. It utilizes heavy structural lines, high-contrast states, and a utilitarian layout inspired by physical control terminals. The interface must evoke a sense of mechanical reliability and real-time responsiveness, ensuring operators can manage complex fleet logistics with absolute confidence.

## Colors

The palette is optimized for high-visibility in dark environments (Dark Mode default). 

- **Primary (Industrial Yellow):** Reserved for active AGV paths, primary action buttons, and critical warnings. It represents movement and energy.
- **Success (Emerald):** Used exclusively for "Ready," "Online," and "Charging" states.
- **Error/E-Stop (Vivid Red):** High-saturation red for emergency stops, fleet collisions, and critical hardware failures.
- **Atmosphere:** The background uses Deep Slate to reduce eye strain during long shifts, while Surface Slate distinguishes modular panels and control decks.

## Typography

Typography is split between **Inter** for general interface legibility and **JetBrains Mono** for all technical data, coordinates, and telemetry. 

- **JetBrains Mono** is used for any value that changes in real-time to prevent "jitter" in the UI, as monospaced characters occupy consistent horizontal space.
- **Inter** handles help text, labels, and descriptions where density and readability are paramount.
- All labels for AGV IDs, battery percentages, and coordinates must use the `data-mono` or `label-caps` styles to emphasize the "machine-readable" nature of the system.

## Layout & Spacing

This design system utilizes a **Rigid Grid** philosophy. Content is organized into a 12-column grid for large dashboards, but shifts to a modular, "instrument cluster" layout for control views.

- **Grid:** 16px gutters and 24px outer margins.
- **Modular Panels:** Every panel must follow a strict 4px baseline grid. Padding within cards is fixed at 20px (`panel-padding`) to maintain a density appropriate for industrial touch-screens or high-res monitors.
- **Mobile/Tablet:** On smaller diagnostic tablets, the layout collapses into a single-column stack. Touch targets for critical actions (E-Stop, Redirect) must be at least 48x48px.

## Elevation & Depth

Depth is conveyed through **Tonal Layering** and **High-Contrast Outlines** rather than soft shadows, which can appear muddy on industrial displays.

- **Base Layer:** `#0F172A` (Deep Slate).
- **Surface Layer:** `#1E293B` (Darker Slate) with a 1px solid border of `#334155`.
- **Active State:** Elements in an "Active" or "Selected" state utilize an outer glow (box-shadow) using the primary color (`#FACC15`) with a 0% spread and 8px blur to simulate a hardware LED indicator.
- **Interactive Elements:** Use 2px solid borders to define clickability. Never use drop shadows for hierarchy; use color value shifts.

## Shapes

The shape language is **Technical and Geometric**. 

- **Corner Radius:** Standard components use a `4px` (Soft) radius to maintain a modern feel while remaining structural. 
- **Industrial Elements:** Progress bars and segmented controls should use `0px` (Sharp) corners for a more "built" or "fabricated" appearance.
- **Icons:** Use 2px stroke widths with sharp joins. Avoid rounded terminals in iconography.

## Components

### Buttons
- **Primary:** Background Industrial Yellow (`#FACC15`), Text Deep Slate (`#0F172A`), Bold Caps.
- **Secondary:** Transparent background, 2px Solid Border (`#94A3B8`), Text White (`#F8FAFC`).
- **Emergency:** Background Red (`#EF4444`), Text White, pulsing animation when active.

### Modular Cards
- Use a 2px top-border accent color to denote status (Yellow for moving, Emerald for idle, Red for error). 
- Headers must be in `label-caps` with a subtle background tint (`#334155`).

### SCADA Gauges & Progress
- **Linear Progress:** Use a segmented "block" style rather than a smooth fill. 10 segments total.
- **Circular Gauges:** High-contrast stroke with "Critical Zones" marked in Red.

### Toggles & Segments
- **Segmented Control:** All-caps labels. The selected segment should have a thick 3px bottom-border in Industrial Yellow.
- **Indicators:** "Glow" dots (8px x 8px) next to labels to indicate real-time heartbeat or connectivity.

### Input Fields
- Dark background (`#0F172A`), 1px border. On focus, the border becomes Industrial Yellow with a subtle inner glow. Use Monospace font for all numeric input.