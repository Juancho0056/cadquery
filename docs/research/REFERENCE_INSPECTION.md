# Inspección real de referencias 3MF — 2026-11-25
Archivos proporcionados por el usuario, analizados localmente sin incorporar mallas al repositorio.
## Procedencia
- Reversible Hand Ratchet, KMM, 2025-03-15, Standard Digital File License, perfil Full PLA One Color. SHA256 fa11125b26dca3ff464bd275249b5c89a528c4e26b47be84e1febe68ce275f97.
- Modular Universal Knuckle Joint System, AbLab, 2026-06-04, MakerWorld Exclusive License. SHA256 0f690df87f7ff85125d816e2d09f2b0589404a47bcfbc3b401f72ebc8e07ff12.
Licencias declaradas en metadatos; permisos de redistribución/modificación no verificados. No copiar geometrías, imágenes ni archivos derivados.
## Método y evidencia
Lectura ZIP/XML de los modelos core y recursos externos Production; se resolvió cada referencia directa de build a la malla hoja y se aplicó la matriz de build a sus vértices. Las transformaciones de componentes de estos dos archivos son identidad. Unidades mm. Recuento de incidencias por arista no orientada y área de triángulos: las 12 mallas tienen cero aristas con incidencia distinta de dos y cero triángulos con norma del producto cruzado inferior a 1e-10 mm². Esto no demuestra ausencia de auto-intersecciones ni validez mecánica.
## Ratchet: siete mallas, escala de impresión 1
| Build ID | Mesh ID | Caja local XYZ mm | Vértices | Triángulos |
|---|---|---|---|---|
|2|1|32.998 × 32.998 × 16|1953|3902|
|4|3|8.889 × 15.220 × 9.100|492|980|
|6|5|8.997 × 12.741 × 7.450|234|464|
|8|7|35.901 × 42.542 × 5|612|1224|
|10|9|45.800 × 74.202 × 5|1572|3148|
|12|11|15.800 × 15.800 × 3|844|1688|
|14|13|49.989 × 81.000 × 13.100|6852|13720|
La miniatura de placa muestra rueda dentada, dos mitades de carcasa y elementos pequeños de conmutación/retención, incluyendo una pieza flexible con dos extremos. La identificación individual de uñas, pivotes y estados exactos requiere inspección adicional. El autor sugiere PETG para el elemento elástico, no certifica el conjunto PETG ni su uso como soporte de carga.
## Knuckle: cinco mallas
| Build ID | Mesh ID | Caja local XYZ mm | Caja de impresión XYZ mm | Vértices | Triángulos |
|---|---|---|---|---|---|
|2|1|239.971 × 50 × 74.948|14.060 × 21.075 × 67.480|7891|15786|
|4|3|45 × 15 × 44.942|12.638 × 12.654 × 4.218|1642|3284|
|6|5|9 × 9 × 45|2.531 × 2.531 × 12.654|477|950|
|8|7|45 × 145 × 44.942|12.638 × 12.654 × 40.774|2123|4246|
|10|9|247.5 × 95 × 75|21.090 × 26.714 × 69.597|11003|22014|
Todas las matrices de build contienen escala uniforme 0.2812 más rotación y traslación. No confundir caja local con tamaño impreso. Metadatos Bambu incluyen assemble_item con matrices de escala inversa aproximada 3.5561878; no se han reconstruido ni validado sus posiciones de montaje. Miniatura muestra horquilla, ojo y pasadores. No inferir diámetros de pivote de las cajas envolventes.
## Decisiones
Diseño independiente: horquilla con doble apoyo, eje estructural separado de la conmutación, dos uñas opuestas y bloqueo positivo independiente. No escalar directamente el ratchet de mano ni sus flexuras; el brazo presenta carga sostenida y riesgo de caída. No adoptar la orientación de impresión del autor sin ensayos de anisotropía.
Estado: inventario real PASS para los recuentos descritos. Interfaces detalladas, cinemática, licencias y capacidad de carga NOT_RUN. No autorizar impresión funcional.
