# Laboratorios
| ID | Tema | Estado |
|---|---|---|
| LAB-001 | RET-001: retención axial anular | Prototipo CAD |
| LAB-002 | ASM-001: cierre elástico de tapas | Verificado geométricamente; ensayo físico pendiente |
| LAB-003 | ASM-002: bayoneta | Pendiente |
| LAB-004 | CMP-001: elasticidad integrada | Pendiente |
| LAB-005 | LCK-001: bloqueo por botón | Pendiente |
| LAB-006 | MAG-001: retorno magnético | Pendiente |

## Reproducción
Instalar CadQuery con `python -m pip install -r requirements.txt`.
Ejecutar `python -m unittest discover -s tests -v`.
Generar los nueve STL y nueve STEP con `python mechanical_patterns/RET-001/model.py`. Los exportados son derivados regenerables y no se versionan en esta fase.

## Registro de ensayo
Registrar versión, material, impresora, orientación, parámetros de laminado, holgura nominal y medida, juego axial, facilidad de montaje, desgaste, fallos y decisión. No declarar capacidad de carga sin ensayo.

ASM-001: generar seis STL y seis STEP mediante `python mechanical_patterns/ASM-001/model.py`.
