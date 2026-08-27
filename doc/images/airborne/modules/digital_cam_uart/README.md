# Digital Camera UART Diagrams

These diagrams document the Paparazzi UART camera link and the MORA companion computer.

| Diagram | GIF | Editable source |
|---|---|---|
| System architecture | `system_architecture.gif` | `system_architecture.dot` |
| Physical and local deployment | `deployment_modes.gif` | `deployment_modes.dot` |
| Shoot/status sequence | `shoot_sequence.gif` | `shoot_sequence.dot` |
| MORA frame structure | `mora_frame.gif` | `mora_frame.dot` |

## Terminology

**MORA** is the companion computer that runs CATIA and receives Paparazzi camera commands. The same term also prefixes the existing serial message identifiers in the protocol implementation.

## Regenerate

From the repository root:

```bash
for source in doc/images/airborne/modules/digital_cam_uart/*.dot; do
  base="${source%.dot}"
  dot -Tpng -Gdpi=150 "$source" -o "${base}.png"
  magick "${base}.png" -colors 256 "${base}.gif"
  rm "${base}.png"
done
```
