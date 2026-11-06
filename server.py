import asyncio
import json
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path

import httpx
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

STATIC = Path(__file__).parent / "static"

ESP32_URL = os.environ.get("ESP32_URL", "http://192.168.1.60")  # IP in ra ở Serial Monitor
VALID = {"GOODS_A": "A", "GOODS_B": "B"}  # nội dung QR -> điểm đến
DEBOUNCE_S = 3.0                          # bỏ qua QR trùng trong 3 giây
POLL_S = 1.0                              # hỏi trạng thái xe mỗi 1 giây (cũng là heartbeat)

clients: set[WebSocket] = set()           # điện thoại + dashboard
# phase / corners lấy từ /status của ESP32 (None khi xe không chạy)
state = {"last": None, "car": "OFFLINE", "phase": None, "corners": None, "log": [],
         "dest": None, "reason": None, "distance": None, "obstacle": False,
         "trips": {"A": 0, "B": 0}}  # trips: chỉ đếm chuyến về tới vạch đích (không tính chuyến lỗi)
last_seen = {"payload": None, "t": 0.0}
http: httpx.AsyncClient | None = None


def add_log(msg: str) -> None:
    entry = {"t": time.strftime("%H:%M:%S"), "msg": msg}
    state["log"] = (state["log"] + [entry])[-50:]


def describe_event(code: str) -> str:
    """Dịch mã sự kiện ASCII của ESP32 sang tiếng Việt.
    Mã: GO:A  SEEN:L:1  TURNED:1  DROP:A  DONE:A  HOME"""
    p = str(code).split(":")
    try:
        if p[0] == "GO":
            return f"Xe xuất phát tới điểm {p[1]}"
        if p[0] == "SEEN":
            return f"Phát hiện góc {'trái' if p[1] == 'L' else 'phải'} thứ {p[2]}"
        if p[0] == "TURNED":
            return f"Đã rẽ xong góc {p[1]}"
        if p[0] == "DROP":
            return f"Đã tới điểm {p[1]}, đang đổ hàng"
        if p[0] == "DONE":
            return f"Đã đổ hàng xong tại điểm {p[1]}, đang quay về vạch đích"
        if p[0] == "HOME":
            return "Xe đã về tới vạch đích"
    except IndexError:
        pass
    return f"Sự kiện: {code}"


async def broadcast() -> None:
    data = json.dumps({"type": "state", **state})
    for ws in list(clients):
        try:
            await ws.send_text(data)
        except Exception:
            clients.discard(ws)


async def car_request(path: str, params: dict | None = None) -> dict | None:
    """Gọi ESP32. Trả về JSON, hoặc None nếu không liên lạc được."""
    try:
        r = await http.get(f"{ESP32_URL}{path}", params=params, timeout=2.0)
        return r.json()
    except Exception:
        return None


async def poll_car() -> None:
    seq_seen: int | None = None  # số thứ tự sự kiện cuối cùng đã ghi vào nhật ký
    while True:
        res = await car_request("/status")
        changed = False

        # 1) Trạng thái xe
        new = res.get("state", "OFFLINE") if res else "OFFLINE"
        if new != state["car"]:
            # Chuyển trạng thái thông thường đã có sự kiện từ ESP32 mô tả,
            # nên chỉ ghi thêm khi mất/có lại kết nối hoặc lỗi.
            if new in ("OFFLINE", "ERROR") or state["car"] == "OFFLINE":
                why = f" ({res['reason']})" if res and res.get("reason") else ""
                add_log(f"Xe: {state['car']} -> {new}{why}")
            state["car"] = new
            changed = True

        # 2) Giai đoạn chuyến đi + số góc đã rẽ
        phase = res.get("phase") if res else None
        corners = res.get("corners") if res else None
        if phase != state["phase"] or corners != state["corners"]:
            state["phase"], state["corners"] = phase, corners
            changed = True

        # 2b) Thông tin thêm cho dashboard: điểm đến, lý do lỗi, siêu âm
        extra = {
            "dest": res.get("dest") if res and res.get("dest") in VALID.values() else None,
            "reason": res.get("reason") if res and new == "ERROR" else None,
            "distance": round(res["distance"]) if res and "distance" in res else None,
            "obstacle": bool(res.get("obstacle")) if res else False,
        }
        if any(state[k] != v for k, v in extra.items()):
            state.update(extra)
            changed = True

        # 3) Sự kiện mới từ ESP32 (đã tới điểm, đổ hàng xong, quay về...)
        if res and "seq" in res:
            seq = int(res["seq"])
            if seq_seen is None:
                seq_seen = seq      # lần đầu kết nối: bỏ qua sự kiện cũ
            elif seq < seq_seen:
                seq_seen = 0        # ESP32 vừa khởi động lại, số thứ tự về 0
            for ev in res.get("events", []):
                if ev.get("id", 0) > seq_seen:
                    add_log(describe_event(ev.get("m", "")))
                    if ev.get("m") == "HOME" and state["last"] in state["trips"]:
                        state["trips"][state["last"]] += 1  # xe về tới đích = hoàn thành 1 chuyến
                    changed = True
            seq_seen = seq

        if changed:
            await broadcast()
        await asyncio.sleep(POLL_S)


@asynccontextmanager
async def lifespan(app: FastAPI):
    global http
    http = httpx.AsyncClient()
    task = asyncio.create_task(poll_car())
    yield
    task.cancel()
    await http.aclose()


app = FastAPI(lifespan=lifespan)


async def handle_scan(payload: str) -> None:
    payload = payload.strip()[:64]
    now = time.time()

    if payload == last_seen["payload"] and now - last_seen["t"] < DEBOUNCE_S:
        return  # quét lặp, bỏ qua
    last_seen.update(payload=payload, t=now)

    if payload not in VALID:
        add_log(f"Mã không hợp lệ: {payload}")
    elif state["car"] == "OFFLINE":
        add_log("ESP32 không phản hồi (OFFLINE), bỏ qua lệnh")
    elif state["car"] != "IDLE":
        add_log(f"Xe đang bận ({state['car']}), bỏ qua {payload}")
    else:
        dest = VALID[payload]
        res = await car_request("/go", {"dest": dest})
        if res and res.get("ok"):
            state["last"] = dest
            state["car"] = res["state"]
            state["phase"] = res.get("phase")
            state["corners"] = res.get("corners")
            add_log(f"Quét {payload} -> gửi lệnh GO, điểm đến {dest}")
        else:
            add_log(f"Xe từ chối lệnh: {res.get('err') if res else 'không phản hồi'}")
    await broadcast()


async def handle_stop() -> None:
    res = await car_request("/stop")
    if res:
        state["car"] = res["state"]
        state["phase"] = res.get("phase")
        state["corners"] = res.get("corners")
        add_log("Đã gửi lệnh DỪNG / đặt lại")
    else:
        add_log("Gửi lệnh DỪNG thất bại: ESP32 không phản hồi")
    await broadcast()


@app.get("/")
def dashboard():
    return FileResponse(STATIC / "dashboard.html")


@app.get("/scan")
def scan_page():
    return FileResponse(STATIC / "scan.html")


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    await ws.send_text(json.dumps({"type": "state", **state}))
    try:
        while True:
            msg = json.loads(await ws.receive_text())
            if msg.get("type") == "scan":
                await handle_scan(str(msg.get("payload", "")))
            elif msg.get("type") == "stop":
                await handle_stop()
    except (WebSocketDisconnect, json.JSONDecodeError):
        pass
    finally:
        clients.discard(ws)
