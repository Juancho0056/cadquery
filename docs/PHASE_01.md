# Fase 01 — ingeniería inversa
Originales requeridos: Knuckle_Joint__v2.3mf y hand+ratchet+pla+only.3mf. No incorporarlos al repositorio público sin permiso. Estado: originales ausentes, ingeniería inversa NOT_RUN.
## Herramienta de inventario
Desde la raíz:
```powershell
python scripts/inspect_3mf.py "references/third_party/Knuckle_Joint__v2.3mf" > artifacts/knuckle_inventory.json
python scripts/inspect_3mf.py "references/third_party/hand+ratchet+pla+only.3mf" > artifacts/ratchet_inventory.json
```
La herramienta calcula SHA256, lista recursos y objetos, unidades, vértices, triángulos, límites locales en mm, referencias de componentes y transformaciones de build sin aplicarlas. No determina dimensiones globales, licencias, topología cerrada, cinemática ni resistencia.
## Criterios de cierre
1. Registrar autor, URL, versión, licencia y SHA256.
2. Reconstruir transformaciones de componentes y placas, identificar piezas y verificar escalas.
3. Medir interfaces, pivotes, corona, uñas, selector y retenciones reales.
4. Identificar trayectoria de carga y modos de fallo.
5. Documentar A/B/C y transición bajo carga.
6. Seleccionar qué reutilizar, componer, extender o crear; registrar ADR de cartucho.
No afirmar que la fase se completó sin originales y evidencia.
