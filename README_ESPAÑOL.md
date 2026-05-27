# ESP32 FTMS Indoor Bike Sensor

## Introducción

Este proyecto transforma una bicicleta estática convencional con resistencia magnética en una bicicleta inteligente compatible con FTMS (Fitness Machine Service) utilizando un ESP32.

El sistema mide la cadencia de pedaleo en tiempo real mediante un sensor magnético tipo reed switch, aplica un algoritmo de filtrado EMA adaptativo para estabilizar las RPM, estima la potencia desarrollada mediante un modelo matemático cuadrático y transmite los datos por Bluetooth Low Energy (BLE) utilizando el estándar oficial FTMS.

El firmware fue diseñado para mejorar la compatibilidad con aplicaciones de ciclismo indoor como MyWhoosh, Zwift, Kinomap y otras plataformas compatibles con FTMS, manteniendo un comportamiento fluido y realista.

---

# Características Principales

- Medición de cadencia en tiempo real mediante sensor magnético
- Filtro EMA adaptativo con ajuste dinámico de α según delta de RPM
- Estimación de potencia mediante modelo cuadrático
- Compatibilidad BLE FTMS Indoor Bike
- Simulación de velocidad para compatibilidad con apps
- Lectura del nivel de resistencia mediante potenciómetro
- Detección automática de frenado y parada
- Envío BLE periódico cada 300 ms

---

# Obtención de la Cadencia Instantánea

Fijado al chasis de la bicicleta se encuentra un sensor magnético (*reed switch*), el cual detecta el paso de un imán instalado en la polea de las bielas.

Cada vez que las bielas completan una vuelta, el imán activa el sensor y genera un pulso eléctrico discreto.

Esto registra una secuencia de eventos en el tiempo:

```text
t₀, t₁, t₂, ..., tₙ
```

Para calcular la cadencia instantánea de pedaleo, el sistema mide el tiempo transcurrido entre el pulso actual (`tₙ`) y el inmediatamente anterior (`tₙ₋₁`):

```text
Δt = tₙ - tₙ₋₁
```

Dado que el tiempo se mide en microsegundos para maximizar la precisión, la conversión a revoluciones por minuto (RPM) se realiza mediante:

```text
Cadencia (RPM) = 60.000.000 / Δt
```

---

# Filtrado de la Cadencia

Enviar directamente la señal instantánea de RPM genera fluctuaciones bruscas y ruido visual dentro de las aplicaciones de ciclismo.

Para suavizar la señal se implementa un filtro EMA (*Exponential Moving Average*).

---

# Filtro EMA

A diferencia de un promedio simple, el EMA otorga mayor peso a las muestras recientes y reduce progresivamente la influencia de las antiguas.

```text
RPMfiltrada(n) = α · RPMinstantánea(n) + (1 - α) · RPMfiltrada(n - 1)
```

Donde:

- `α` es el factor de suavizado (`0 < α < 1`)
- α alto → respuesta rápida pero más ruido
- α bajo → señal más estable pero más lenta

---

# Mejora Adaptativa (α según Delta)

Para evitar tener que elegir entre estabilidad o rapidez, el firmware ajusta dinámicamente el valor de `α` según la magnitud del cambio de RPM.

Primero se calcula la diferencia absoluta entre la pedalada actual y la anterior:

```text
δₙ = | RPMinstantánea(n) - RPMinstantánea(n - 1) |
```

Después, el sistema adapta automáticamente el valor de α:

| Delta RPM | Comportamiento |
|---|---|
| `δₙ < 3` | Máximo suavizado → `α = 0.12` |
| `3 ≤ δₙ < 10` | Transición progresiva |
| `10 ≤ δₙ < 20` | Respuesta rápida |
| `δₙ ≥ 20` | Respuesta instantánea → `α = 0.95` |

Interpolaciones utilizadas:

```text
α = 0.12 + ((δₙ - 3) / 7) · 0.23
```

```text
α = 0.35 + ((δₙ - 10) / 10) · 0.45
```

---

# Suavizado Interno de α

Para evitar cambios bruscos en el propio factor de suavizado, α también pasa por un segundo filtro EMA interno:

```text
αsmooth(n) = 0.70 · α(δₙ) + 0.30 · αsmooth(n - 1)
```

Esto mejora significativamente la estabilidad visual manteniendo una respuesta rápida durante aceleraciones o sprints.

---

# Estimación de Potencia

La bicicleta no dispone de sensores físicos de torque o galgas extensiométricas.

Por ello, la potencia desarrollada por el usuario se estima matemáticamente combinando:

- La cadencia filtrada (RPM)
- El nivel de resistencia magnética seleccionado manualmente

---

# Principio de Resistencia Magnética

El sistema de frenado funciona mediante Corrientes de Foucault (*Eddy Currents*).

Al pedalear, el volante metálico atraviesa el campo magnético generado por imanes permanentes. Esto induce corrientes microscópicas dentro del volante, creando un campo magnético opuesto al movimiento y generando resistencia sin contacto físico.

El usuario modifica manualmente la intensidad de frenado alterando la distancia entre imanes y volante.

---

# Modelo Matemático de Potencia

La fuerza de oposición magnética aumenta aproximadamente de forma lineal con la velocidad del volante.

Sin embargo, dado que la potencia es fuerza × velocidad, el esfuerzo requerido crece de forma cuadrática.

El firmware implementa el siguiente modelo:

```cpp
Potencia (W) = (K · RPM²) + (C · RPM)
```

Donde:

- `K` → componente cuadrático asociado al freno magnético
- `C` → componente lineal asociado a pérdidas mecánicas

---

# Lectura del Nivel de Resistencia

La bicicleta incorpora un potenciómetro físico conectado al selector de resistencia.

El ESP32 lee este valor mediante su ADC de 12 bits:

```text
Rango ADC: 0 → 4095
```

Para reducir ruido eléctrico, el firmware promedia 8 muestras consecutivas antes de convertirlas en 8 niveles discretos de resistencia.

```cpp
Nivel = constrain( ((ADC - 120) / (3980 - 120) * 8) + 1, 1, 8 )
```

---

# Tabla Dinámica de Coeficientes

Cada nivel utiliza coeficientes `K` y `C` específicos:

| Nivel | K | C |
|---|---|---|
| 1 | 0.006 | 0.05 |
| 2 | 0.009 | 0.07 |
| 3 | 0.012 | 0.10 |
| 4 | 0.017 | 0.14 |
| 5 | 0.022 | 0.18 |
| 6 | 0.030 | 0.24 |
| 7 | 0.039 | 0.30 |
| 8 | 0.050 | 0.38 |

---

# Ejemplo de Cálculo

Ejemplo:

- Nivel de resistencia: 4
- Cadencia: 80 RPM
- `K = 0.017`
- `C = 0.14`

Componente cuadrático:

```text
0.017 × 80² = 108.8 W
```

Componente lineal:

```text
0.14 × 80 = 11.2 W
```

Potencia total estimada:

```text
120 W
```

---

# Implementación BLE FTMS

El ESP32 implementa el perfil oficial FTMS definido por Bluetooth SIG.

Cada 300 ms se envía automáticamente un paquete Indoor Bike Data mediante notificaciones BLE.

---

# Estructura del Paquete FTMS

El paquete contiene tres campos principales declarados mediante la máscara `0x0044`:

| Bytes | Campo |
|---|---|
| 0–1 | Flags |
| 2–3 | Velocidad simulada |
| 4–5 | Cadencia |
| 6–7 | Potencia |

Todos los datos se transmiten en formato *little-endian*.

---

# Codificación de la Cadencia

El estándar FTMS utiliza resolución de 0.5 RPM.

Por ello, la cadencia se transmite como:

```cpp
cadenceValue = cadence × 2
```

La aplicación divide internamente el valor entre dos al recibirlo.

---

# Simulación de Velocidad

Algunas aplicaciones rechazan paquetes FTMS si el campo velocidad no existe.

Como la bicicleta no dispone de sensor de velocidad real, esta se deriva artificialmente a partir de la cadencia:

```cpp
speedValue = cadence × 30
```

Esto mejora la compatibilidad con aplicaciones de terceros.

---

# BLE Notify vs Read

El ESP32 utiliza el mecanismo `notify` de BLE.

Una vez que la aplicación se suscribe a la característica `Indoor Bike Data`, el ESP32 transmite automáticamente nuevos paquetes cada 300 ms sin necesidad de solicitudes manuales.

Si no existe un dispositivo conectado, el envío se omite automáticamente.
