# Harness H01–H10
| ID | Gate | Cobertura inicial |
|---|---|---|
| H01 | sólido válido, volumen, número de sólidos, malla cerrada | sólido; malla pendiente |
| H02 | dimensiones, interfaces, espesor, dientes | envolvente/altura; paso derivado de configuración |
| H03 | interferencias, posiciones y barridos | NOT_RUN |
| H04 | orificios, recorridos y roscas | paso central; resto NOT_RUN |
| H05 | cinemática reversible, bloqueo y seguro | NOT_RUN |
| H06 | ensamblabilidad y mantenimiento | NOT_RUN |
| H07 | fabricación FDM y tolerancias | NOT_RUN |
| H08 | resistencia, fatiga, fluencia | NOT_RUN |
| H09 | STEP/STL/3MF, trazabilidad | STEP/STL; 3MF NOT_RUN |
| H10 | evidencia y SHA | reporte JSON; SHA si disponible |
Cada gate debe registrar evidencia, parámetros, versión, esperado/obtenido, estado PASS/FAIL/NOT_RUN y commit. No declarar listo para prototipado sin interfaces, montaje, exportaciones, orientación y riesgos. Carga de 1 kg exige ensayos físicos.
