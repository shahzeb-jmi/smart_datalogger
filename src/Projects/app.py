from flask import Flask, request, jsonify, render_template, redirect, url_for, session, send_file
import mysql.connector
import os
from datetime import datetime, timedelta
import pandas as pd
from config import *    # expects DB_HOST, DB_USER, DB_PASS, DB_NAME

app = Flask(__name__)
app.secret_key = "super_secret_key"

# -------------------------
# Database connection
# -------------------------
def get_db():
    return mysql.connector.connect(
        host=DB_HOST,
        user=DB_USER,
        password=DB_PASS,
        database=DB_NAME
    )

# -------------------------
# Helper: safe DataFrame from rows
# -------------------------
def rows_to_df(rows, columns=None):
    # rows is list[dict] from cursor(dictionary=True)
    if not rows:
        if columns:
            return pd.DataFrame(columns=columns)
        return pd.DataFrame()
    return pd.DataFrame(rows)

# -------------------------
# ESP32 DATA ENDPOINT
# -------------------------
@app.route("/data", methods=["POST"])
def data():
    try:
        payload = request.get_json(force=True)

        device_id = payload.get("device_id")
        ts_ms = payload.get("ts_ms")

        # convert ms -> human timestamp (IST)
        if ts_ms is None:
            timestamp = datetime.now()
        else:
            timestamp = datetime.utcfromtimestamp(int(ts_ms) / 1000) + timedelta(hours=5, minutes=30)
        timestamp_str = timestamp.strftime('%Y-%m-%d %H:%M:%S')

        sensors = payload.get("sensors", {})
        temp = sensors.get("temperature_c", 0) or 0
        humidity = sensors.get("humidity_pct", 0) or 0
        ldr_raw = sensors.get("ldr_raw", None)

        # accel
        accel = sensors.get("accel_g", {})
        acc_x = accel.get("x", 0) or 0
        acc_y = accel.get("y", 0) or 0
        acc_z = accel.get("z", 0) or 0

        # light %
        if ldr_raw is None:
            light = None
        else:
            try:
                light = (float(ldr_raw) / 4095.0) * 100.0
            except Exception:
                light = None

        db = get_db()
        c = db.cursor()

        c.execute("""
            INSERT INTO sensor_data
            (device_id, ts_ms, timestamp, temp, humidity, ldr_raw, light, acc_x, acc_y, acc_z)
            VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)
        """, (
            device_id,
            ts_ms,
            timestamp_str,
            float(temp) if temp is not None else 0.0,
            float(humidity) if humidity is not None else 0.0,
            ldr_raw,
            light,
            float(acc_x),
            float(acc_y),
            float(acc_z)
        ))

        db.commit()
        c.close()
        db.close()

        return jsonify({"status": "OK"}), 200

    except Exception as e:
        # print error to console for debugging
        print("ERR /data:", repr(e))
        return jsonify({"status": "error", "error": str(e)}), 400

# -------------------------
# LOGIN (simple)
# -------------------------
@app.route("/login", methods=["GET", "POST"])
def login():
    if request.method == "POST":
        if request.form.get("username") == "admin" and request.form.get("password") == "1234":
            session["logged"] = True
            return redirect(url_for("index"))
        return render_template("login.html", error="Invalid login")
    return render_template("login.html")

# -------------------------
# MAIN DASHBOARD
# -------------------------
@app.route("/")
def index():
    if "logged" not in session:
        return redirect(url_for("login"))

    db = get_db()
    c = db.cursor(dictionary=True)
    c.execute("SELECT * FROM sensor_data ORDER BY timestamp DESC LIMIT 200")
    rows = c.fetchall()
    c.close()
    db.close()

    df = rows_to_df(rows, columns=[
        "id","device_id","ts_ms","timestamp","temp","humidity","ldr_raw","light","acc_x","acc_y","acc_z","magnitude"
    ])

    # ensure numeric columns exist to avoid KeyError
    for col in ["temp","humidity","light","acc_x","acc_y","acc_z","magnitude"]:
        if col not in df.columns:
            df[col] = pd.Series(dtype=float)

    stats = {
        "temp_min": None if df.empty else float(df["temp"].min()),
        "temp_max": None if df.empty else float(df["temp"].max()),
        "temp_mean": None if df.empty else float(df["temp"].mean()),
        "temp_std": None if df.empty else float(df["temp"].std()),

        "humidity_min": None if df.empty else float(df["humidity"].min()),
        "humidity_max": None if df.empty else float(df["humidity"].max()),
        "humidity_mean": None if df.empty else float(df["humidity"].mean()),
        "humidity_std": None if df.empty else float(df["humidity"].std()),

        "acc_x_min": None if df.empty else float(df["acc_x"].min()),
        "acc_x_max": None if df.empty else float(df["acc_x"].max()),
        "acc_x_mean": None if df.empty else float(df["acc_x"].mean()),
        "acc_x_std": None if df.empty else float(df["acc_x"].std()),

        "acc_y_min": None if df.empty else float(df["acc_y"].min()),
        "acc_y_max": None if df.empty else float(df["acc_y"].max()),
        "acc_y_mean": None if df.empty else float(df["acc_y"].mean()),
        "acc_y_std": None if df.empty else float(df["acc_y"].std()),

        "acc_z_min": None if df.empty else float(df["acc_z"].min()),
        "acc_z_max": None if df.empty else float(df["acc_z"].max()),
        "acc_z_mean": None if df.empty else float(df["acc_z"].mean()),
        "acc_z_std": None if df.empty else float(df["acc_z"].std()),

        "magnitude_min": None if df.empty else float(df["magnitude"].min()) if "magnitude" in df.columns else None,
        "magnitude_max": None if df.empty else float(df["magnitude"].max()) if "magnitude" in df.columns else None,
        "magnitude_mean": None if df.empty else float(df["magnitude"].mean()) if "magnitude" in df.columns else None,
        "magnitude_std": None if df.empty else float(df["magnitude"].std()) if "magnitude" in df.columns else None,

        "light_min": None if df.empty else float(df["light"].min()),
        "light_max": None if df.empty else float(df["light"].max()),
        "light_mean": None if df.empty else float(df["light"].mean()),
        "light_std": None if df.empty else float(df["light"].std()),
    }

    return render_template("index.html", table=df.to_dict(orient="records"), stats=stats)

# -------------------------
# EXPORT CSV
# -------------------------
@app.route("/export")
def export():
    os.makedirs("exports", exist_ok=True)
    today_str = datetime.now().strftime("%Y-%m-%d")
    export_path = f"exports/{today_str}.csv"

    db = get_db()
    c = db.cursor(dictionary=True)
    c.execute("SELECT * FROM sensor_data ORDER BY timestamp DESC")
    rows = c.fetchall()
    c.close()
    db.close()

    df = rows_to_df(rows)
    # safe: if empty, create CSV with headers only
    if df.empty:
        # choose reasonable headers
        df = pd.DataFrame(columns=[
            "id","device_id","ts_ms","timestamp","temp","humidity","ldr_raw","light","acc_x","acc_y","acc_z","magnitude"
        ])
    df.to_csv(export_path, index=False)

    return send_file(export_path, as_attachment=True)

# -------------------------
# API FOR CHART.JS (returns JSON list)
# -------------------------
@app.route("/api/latest")
def api_latest():
    db = get_db()
    c = db.cursor(dictionary=True)
    c.execute("SELECT * FROM sensor_data ORDER BY timestamp DESC LIMIT 200")
    rows = c.fetchall()
    c.close()
    db.close()
    # return list of dicts
    return jsonify(rows)

# -------------------------
# Run server
# -------------------------
if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
