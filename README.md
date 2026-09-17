# Gravity Simulation

Simulasi gravitasi N-body 3D dalam C++. Matahari dan semua delapan planet (Merkurius, Venus, Bumi, Mars, Jupiter, Saturnus, Uranus, Neptunus) saling tarik berdasarkan hukum gravitasi Newton dan ditampilkan sebagai bola 3D dengan lighting dan GLSL shader.

## Build

```bash
g++ -std=c++17 -Wall -Wextra -pedantic -O2 main.cpp $(pkg-config --cflags --libs glfw3 epoxy) -o gravity_sim
```

## Jalankan

```bash
./gravity_sim
```

Jalankan dari sesi desktop Linux yang memiliki Wayland atau X11:

```bash
./gravity_sim
```

Kontrol di dalam window:

- `Space`: pause/resume
- `R`: reset simulasi
- `F11`: toggle fullscreen
- `Esc` atau `Q`: keluar

Saat mulai, program langsung masuk fullscreen. Renderer menggunakan sphere mesh, kamera perspektif, orbit 3D, diffuse lighting, rim light, dan emissive shader untuk Matahari.
