# ASM-001 — Cierre elástico desmontable (cantilever snap-fit)

## Problema y principio
Cerrar una carcasa sin tornillos ni adhesivos. Dos pestañas integradas en la tapa flexionan hacia el interior al introducirse; su rampa facilita el montaje y el hombro horizontal se engancha en ventanas laterales. Para abrir, presionar ambos ganchos hacia el interior a través de las ventanas con una herramienta roma si los dedos no alcanzan, y elevar la tapa. No forzar.

## Parámetros (mm)
Carcasa 40 × 30 × 20; paredes 3; fondo 2; tapa 2.4; dedos 14 × 8 × 1.6; enganche nominal 1.0; holguras 0.20, 0.30, 0.40. Flexión lateral nominal requerida: enganche + holgura adicional de montaje (aproximadamente 1.0–1.4 mm según trayectoria). La raíz rectangular es una simplificación de laboratorio; radios y fatiga requieren una iteración posterior.

## Validación geométrica
CadQuery 2.8.0: tres variantes, un sólido válido por pieza, ausencia de interferencia en reposo y contacto bloqueante al trasladar la tapa 1 mm hacia arriba sin deformación. Exportación de seis STL y seis STEP comprobada. Esto no demuestra montaje elástico ni resistencia.

## Laboratorio físico
Usar PETG como hipótesis inicial. Imprimir primero la variante 0.30 mm. El STL de tapa está invertido para apoyar su cara exterior. Revisar orientación de capas y soportes de ventanas y ganchos en el laminador; la orientación de impresión propuesta puede debilitar las raíces por anisotropía. Medir dimensiones, observar blanqueamiento y grietas, registrar esfuerzo de inserción y extracción, juego y ciclos (1, 10, 50). Detener ante daño. No aplicar cargas estructurales.
Comparar con 0.20 y 0.40 mm, y documentar resultados antes de modificar parámetros de espesor o enganche.

## Ejecutar
Desde raíz: `python mechanical_patterns/ASM-001/model.py`. Genera tres carpetas de holgura con dos STL y dos STEP cada una. Ejecutar `python -m unittest discover -s tests -v`.

## Reutilización
Tapas de electrónica, alojamientos de imanes, cubiertas de mantenimiento. No es una reproducción del cierre del brazo ni un cierre estructural certificado.
