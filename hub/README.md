# Notification hub

Boards send events here, and the hub forwards them to Telegram. The bot credentials
stay in one place: each board only needs the address of the hub.

The hub has no authentication and assumes a trusted local network. Never expose its
port to the internet.

## Dashboard

`http://<hub>:8787/` shows a tile per board, for a phone: waiting for you (orange, first),
printing (blue, filled to the percentage), paused, idle (green), error (red), printer not
connected (grey outline) and board offline (grey, with a button to forget it). A tap opens
the board's `/print` page. The page can be added to the home screen.

## Settings

The hub reads `telegram.env` from the repository root (or the path given with `--env`):

```bash
TELEGRAM_BOT_TOKEN=123456789:AAExampleTokenValue
TELEGRAM_CHAT_ID=-1001234567890
PRINTER_DOMAIN=lan            # links become http://<board name>.lan/print; empty = board IP
# TELEGRAM_THREAD_ID=5        # topic, for groups with topics
# HUB_PORT=8787
```

The file is ignored by git. Keep it private: `chmod 600 telegram.env`.

## Running

The hub needs Python 3.9 or newer and nothing else:

```bash
python3 hub/hub.py
```

### Docker

Copy `hub.py`, `Dockerfile`, `.dockerignore`, `compose.yml` and `telegram.env` to one
directory, then:

```bash
chmod 600 telegram.env
echo "FDMHUB_UID=$(id -u)" > .env     # the container reads telegram.env as this user
mkdir -p data                         # the list of printers is kept here
docker compose up -d --build
docker compose logs -f
```

The token is not stored in the image: `telegram.env` is mounted read-only at runtime.

### systemd

Edit the paths and user in `fdmhub.service` and install it:

```bash
sudo cp hub/fdmhub.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now fdmhub
```

## Connecting a board

On each board, set **Settings → Notifications → Hub URL**, for example
`http://192.168.3.2:8787`, and press **Save**. The board must have a route to that
address. **Send test** on the same page checks the whole chain.

Without a URL, prints still stop on `M0`/`M1`, but no notification is sent.

## API

`POST /api/v1/events` answers 200 once Telegram has accepted the message, and 502 if it
did not, so that the board can retry. A repeated `event_id` is not sent again.

```json
{
  "event_id": "e072a1d5606c-1a2b3c4d5e6f7a8b",
  "type": "hold",
  "device": { "id": "e072a1d5606c", "name": "fdm03", "ip": "192.168.3.131" },
  "message": "Check nozzle height",
  "filename": "benchy.gcode"
}
```

`type` is `hold`, `hold_timeout` or `test`. `GET /healthz` reports that the hub is running.

### Status

Each board with a hub URL sends `POST /api/v1/status` every `interval_s` seconds, and at
once when its state changes:

```json
{
  "device": { "id": "e072a1d5606c", "name": "fdm03", "ip": "192.168.3.131" },
  "version": "2.3", "interval_s": 10,
  "state": "printing",
  "filename": "benchy.gcode", "preparing": false, "progress": 42.5, "time_left_s": 600,
  "hold_msg": "Check nozzle height",
  "hotend": 210, "hotend_target": 210, "bed": 60, "bed_target": 60
}
```

`state` is `idle`, `printing`, `paused`, `hold` (waiting for the user), `error` or
`no_printer` (the board is up, the printer is not connected). Unknown fields are dropped.

`GET /api/v1/printers` lists every board that has reported, sorted by name. A board that
has missed three reports is `"online": false` with `"state": "offline"`, and keeps only its
name, address, link and `seen_s_ago`. Started with `--state <file>` (as in `compose.yml`),
the hub saves names, addresses and when each board was last seen, so boards that are off
still show after a restart.

## Tests

```bash
python3 hub/test_hub.py
```
