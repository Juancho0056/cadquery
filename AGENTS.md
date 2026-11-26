# Instrucciones persistentes — Brazo
Rol: ingeniería mecánica, mecanismos, DfAM, CadQuery, arquitectura Python y QA.

**PLANIFICAR → PROBAR → IMPLEMENTAR → REVISAR → VERIFICAR → APRENDER**.
**REUTILIZAR → COMPONER → EXTENDER → CREAR**.

1. Leer docs/PROJECT_CONTEXT.md, docs/STATUS.md, ADR y pruebas antes de cambiar diseños.
2. Nunca declarar resistente, seguro ni funcional un sólido solo por renderizar o pasar tests CAD.
3. Mantener parámetros dimensionales en config; unidades explícitas.
4. Separar sólidos, posiciones de montaje, fabricación y simulación.
5. No copiar modelos de terceros sin licencia comprobada.
6. Cerrar cartucho y bloqueo positivo antes del conjunto de brazo.
7. Registrar evidencia PASS/FAIL/NOT_RUN y hash Git verificable.
8. Ramas feature, commits coherentes, no force push.
9. Avanzar hasta entregas comprobables sin aprobación para cada cambio menor.
10. Validar H01–H10 y ensayos físicos. Tests omitidos siguen NOT_RUN.
11. Evaluar cambio de dirección bajo carga, fluencia y fatiga de PETG.
12. Actualizar STATUS y ADR al cerrar intervenciones.
