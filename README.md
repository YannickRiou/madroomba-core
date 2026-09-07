# madroomba-core

Cerveau du MadRoomba : un Roomba 500 augmenté d'un Raspberry Pi Zero (port
SCI), d'un lidar HLS-LFCD-LDS2 (LDS-01) et d'une Pi Camera, pour s'approcher
des robots « nouvelle génération » : cartographie temps réel, téléopération,
télémétrie.

```
                    ┌────────────────────────────────────────────┐
   navigateur ◄─────┤  madroombad (Pi Zero embarqué)             │
   UI web / SSE     │  ├─ robot.Real ── SCI 115200 ── Roomba 500 │
   carte / téléop   │  ├─ lidar ────── USB 230400 ── LDS-01      │
                    │  ├─ odom (paquets 19/20) ─┐                │
                    │  ├─ mapping (grille log-odds + scan match) │
                    │  └─ camera (libcamera-vid → MJPEG)         │
                    └────────────────────────────────────────────┘
   navigateur ◄─── basestationd (Pi Zero de la base de charge)
   flux dock        └─ camera + détection de présence (diff d'images)
```

## Composants

| Dossier | Rôle |
|---|---|
| `cmd/madroombad` | démon embarqué : fusion odométrie + lidar → carte, HTTP/SSE, téléop, caméra |
| `cmd/basestationd` | démon de la base : caméra du dock en MJPEG + détection de mouvement |
| `internal/lidar` | driver LDS-01 (trames 42 octets, 6 mesures de 6 octets, 1°/mesure) |
| `internal/odom` | intégration des paquets OI 19 (distance) et 20 (angle) en pose 2D |
| `internal/mapping` | grille d'occupation log-odds, ray casting Bresenham, scan matching hill-climbing |
| `internal/robot` | interface `Driver` : implémentation réelle (roomba-go-lib) et simulateur |
| `internal/sim` | monde virtuel (murs + raycast), robot différentiel bruité — dev sans matériel |
| `internal/camera` | MJPEG via rpicam-vid / libcamera-vid / raspivid + détecteur de mouvement |
| `internal/server` | HTTP : UI embarquée (go:embed), SSE `/api/events`, `/api/drive`, `/api/cmd`, `/api/map.png`, `/camera` |

La lib OI vit dans `../roomba-go-lib` (directive `replace` dans `go.mod`).

## Essayer sans matériel

```sh
cd madroomba-core
go run ./cmd/madroombad -sim
# → http://localhost:8080  (ZQSD/WASD/flèches pour piloter)
```

Le simulateur fait tourner un appartement 8×6 m avec cloison et meubles ;
l'odométrie est volontairement bruitée pour exercer le scan matching.

## Câblage (robot)

- **Roomba SCI (mini-DIN 7)** → UART GPIO du Pi (`/dev/serial0`, 115200 bauds).
  ⚠ le SCI est en 5 V TTL, le Pi en 3,3 V : **level shifter obligatoire**
  sur RXD du Pi. Désactiver la console série (`raspi-config`) et le Bluetooth
  (`dtoverlay=disable-bt`) pour libérer le PL011.
- **Pin BRC** du SCI → un GPIO libre (ex. GPIO 17) pour réveiller le robot
  (`-brc-gpio 17`).
- **LDS-01** → adaptateur USB-série (`/dev/ttyUSB0`, 230400 bauds). Le mini-UART
  du Pi Zero est peu fiable à cette vitesse, l'USB est plus sûr.
- **Alimentation** : le SCI fournit la tension batterie (~16 V) ; prévoir un
  buck 5 V (≥1,5 A avec lidar + caméra).
- Orientation lidar : ajuster `-lidar-yaw` (offset en degrés) et
  `-lidar-invert` selon le montage.

## Déploiement

```powershell
cd madroomba-core
.\deploy\build.ps1                     # binaires ARMv6 dans dist/
scp dist/madroombad pi@roomba:/home/pi/
scp dist/basestationd pi@base:/home/pi/
scp deploy/*.service pi@…:/tmp/        # puis sudo mv /etc/systemd/system/
# sudo systemctl enable --now madroombad   (resp. basestationd)
```

## API

- `GET /` — UI web (carte live, téléop, caméra, batterie)
- `GET /api/events` — SSE : évènement `state` (5 Hz : pose, scan, batterie,
  rpm) et `map` (0,5 Hz : PNG base64 + géométrie)
- `POST /api/drive` — `{"left":mm_s,"right":mm_s}` (dead-man : arrêt auto
  après 1 s sans commande)
- `POST /api/cmd` — `{"action":"stop|clean|dock|safe|reset_map|reset_pose"}`
- `GET /api/map.png` — snapshot de la grille d'occupation
- `GET /camera` — MJPEG

Base de charge (`basestationd`, port 8081) : `GET /` (page), `GET /camera`,
`GET /api/status` (`motion`, `score`, `last_motion`, `uptime`).

## Choix techniques

- **Odométrie** : paquets OI 19/20 interrogés à 10 Hz par `QueryList` (les
  lire remet leurs compteurs à zéro, ce qui donne des deltas propres). Le
  paquet 20 est en degrés entiers — précision limitée, d'où le scan matching.
- **Scan matching** : hill-climbing (x, y, θ) sur le score des extrémités de
  rayons contre la grille, 3 pas de recherche coarse-to-fine, correction
  bornée (30 cm / 12°) pour ne jamais « téléporter » le robot. Suffisamment
  léger pour un Pi Zero (ARMv6 mono-cœur) à 5 scans/s.
- **SSE plutôt que WebSocket** : uniquement la stdlib, largement suffisant
  pour de la télémétrie descendante + POST montants.
- **Mode Safe par défaut** (pas Full) : on garde les sécurités falaise/roues.

## Roadmap

- [ ] **ArUco sur la base** : la caméra du dock détecte un marqueur sur le
      capot du robot → pose absolue près de la base pour recaler l'odométrie
      et fermer la boucle en fin de mission (OpenCV/Python sur le Pi de la
      base, envoi de la correction à `madroombad`).
- [ ] Sauvegarde/rechargement de la carte entre sessions.
- [ ] Exploration autonome (frontier-based) au-dessus de la carte.
- [ ] Notification « robot bien docké » via la détection de présence de la base.
- [ ] Encodeurs bruts (paquets 43/44) à la place de 19/20 pour une odométrie
      plus fine.
