"use strict";

// This page only talks to the local demo's /rpc proxy. Credentials stay in Python.
const arms = document.getElementById("arms");
const jogEnabled = document.getElementById("jog-enabled");
const jogStatus = document.getElementById("jog-status");
const activityText = document.getElementById("activity-text");
const connectionStatus = document.getElementById("connection-status");
const connectionDetail = document.getElementById("connection-detail");
const connectionDot = document.getElementById("connection-dot");
const jointNames = ["s0", "s1", "e0", "e1", "w0", "w1", "w2"];
const radiansToDegrees = 180 / Math.PI;
const views = {};
let state = null;
let lastStateAt = 0;
let connected = false;
let commandPending = false;
let operationId = null;
let requestNumber = 0;
let commandVersion = 0;
let screenImageReady = false;
let screenUploadVersion = 0;

function element(tag, className, text, parent) {
    const node = document.createElement(tag);
    if (className) node.className = className;
    if (text !== undefined) node.textContent = text;
    if (parent) parent.appendChild(node);
    return node;
}

function addJogButtons(parent, limb, kind, axis) {
    const buttons = element("div", "jog-buttons", undefined, parent);
    for (const direction of [-1, 1]) {
        const button = element("button", "jog-button", direction < 0 ? "−" : "+", buttons);
        button.type = "button";
        button.disabled = true;
        button.dataset.limb = limb;
        button.dataset.kind = kind;
        button.dataset.axis = axis || "";
        button.dataset.direction = direction;
        const label = "Jog " + limb + " " + (axis || "gripper") + (direction < 0 ? " down" : " up");
        button.setAttribute("aria-label", label);
        button.title = label;
    }
}

// Build the two identical arm panels once; polling only updates their values.
for (const limb of ["left", "right"]) {
    const view = {joints: {}, axes: {}};
    views[limb] = view;
    const panel = element("section", "arm", undefined, arms);
    panel.setAttribute("aria-label", limb + " arm");
    const header = element("div", "arm-header", undefined, panel);
    const title = element("div", "arm-title", undefined, header);
    element("span", "arm-letter", limb[0].toUpperCase(), title);
    const titleText = element("div", "", undefined, title);
    element("h3", "", limb === "left" ? "Left arm" : "Right arm", titleText);
    element("small", "", "7 joints · electric gripper", titleText);
    view.status = element("span", "arm-status", "Waiting", header);

    const presets = element("div", "arm-presets", undefined, panel);
    element("span", "", "Move to", presets);
    for (const pose of ["neutral", "resting"]) {
        const button = element("button", "pose-button", pose === "neutral" ? "Neutral" : "Resting", presets);
        button.type = "button";
        button.disabled = true;
        button.dataset.limb = limb;
        button.dataset.pose = pose;
        button.setAttribute("aria-label", "Move " + limb + " arm to " + pose + " pose");
    }

    const joints = element("div", "section", undefined, panel);
    const jointTitle = element("div", "section-title", undefined, joints);
    element("h4", "", "Joint state", jointTitle);
    element("span", "", "Live position & measured effort", jointTitle);
    const table = element("div", "joint-table", undefined, joints);
    for (const label of ["Joint", "Angle / °", "Torque / N·m", "Jog"]) {
        element("span", "table-label", label, table);
    }
    for (const joint of jointNames) {
        element("span", "joint-name", joint.toUpperCase(), table);
        const angle = element("div", "joint-cell", undefined, table);
        const angleText = element("span", "reading", "—", angle);
        const angleBar = element("span", "", undefined, element("div", "meter", undefined, angle));
        const torque = element("div", "joint-cell", undefined, table);
        const torqueText = element("span", "reading", "—", torque);
        const torqueBar = element("span", "", undefined, element("div", "meter torque-meter", undefined, torque));
        view.joints[joint] = {angleText, angleBar, torqueText, torqueBar};
        addJogButtons(table, limb, "joint", joint);
    }
    element("p", "scale-note", "Bar scales: angle ±180° · torque ±20 N·m. These are display scales, not limits.", joints);

    const pose = element("div", "section", undefined, panel);
    const poseTitle = element("div", "section-title", undefined, pose);
    element("h4", "", "End effector", poseTitle);
    element("span", "", "Base frame", poseTitle);
    const poseGrid = element("div", "pose-grid", undefined, pose);
    for (const axes of [["x", "y", "z"], ["roll", "pitch", "yaw"]]) {
        const column = element("div", "", undefined, poseGrid);
        for (const axis of axes) {
            const row = element("div", "axis-row", undefined, column);
            element("span", "axis-name", axis.length === 1 ? axis.toUpperCase() : axis, row);
            view.axes[axis] = element("span", "reading", "—", row);
            addJogButtons(row, limb, "endpoint", axis);
        }
    }
    view.quaternion = element("p", "quaternion", "Quaternion [w, x, y, z]: —", pose);

    const gripper = element("div", "section", undefined, panel);
    const gripperTitle = element("div", "section-title", undefined, gripper);
    element("h4", "", "Gripper", gripperTitle);
    view.gripperStatus = element("span", "", "Waiting", gripperTitle);
    const gripperContent = element("div", "gripper-content", undefined, gripper);
    const position = element("div", "", undefined, gripperContent);
    element("span", "gripper-label", "Opening · 0 closed / 100 open", position);
    view.gripperPosition = element("span", "reading gripper-reading", "—", position);
    view.gripperPositionBar = element("span", "", undefined, element("div", "gripper-meter", undefined, position));
    const force = element("div", "", undefined, gripperContent);
    element("span", "gripper-label", "Measured force", force);
    view.gripperForce = element("span", "reading gripper-reading", "—", force);
    view.gripperForceBar = element("span", "", undefined, element("div", "gripper-meter gripper-force", undefined, force));
    addJogButtons(gripperContent, limb, "gripper");
}

async function rpc(method, params) {
    const id = String(++requestNumber);
    const abort = new AbortController();
    const timeout = setTimeout(() => abort.abort(), 5000);
    try {
        const response = await fetch("/rpc", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({jsonrpc: "2.0", id, method, params: params || {}}),
            signal: abort.signal,
            cache: "no-store"
        });
        const message = await response.json();
        if (message.error) throw new Error(message.error.message || "Robot request failed");
        if (!response.ok) throw new Error("HTTP " + response.status);
        if (message.id !== id) throw new Error("Unexpected response ID");
        return message.result;
    } catch (error) {
        if (error.name === "AbortError") throw new Error("Request timed out; the robot may still be moving. Use Stop all motion.");
        throw error;
    } finally {
        clearTimeout(timeout);
    }
}

function showActivity(message, isError) {
    activityText.textContent = message;
    activityText.parentElement.classList.toggle("error", !!isError);
    document.getElementById("activity-icon").textContent = isError ? "!" : "○";
}

function updateControls() {
    const fresh = connected && Date.now() - lastStateAt < 1500;
    const feedbackStale = !!(state && state.feedback_stale);
    const fault = state && state.fault;
    const busy = commandPending || operationId !== null || !!(state && state.commands_busy);
    if (feedbackStale || fault) jogEnabled.checked = false;
    jogEnabled.disabled = !fresh || feedbackStale || !!fault;
    const canJog = jogEnabled.checked && fresh && !feedbackStale && !fault && !busy;
    for (const button of document.querySelectorAll(".jog-button, .pose-button")) button.disabled = !canJog;
    const headFresh = state && state.head && !state.head.feedback_stale && Number.isFinite(state.head.pan_rad);
    for (const button of document.querySelectorAll(".head-command")) {
        button.disabled = !canJog || (button.dataset.headMotion === "true" && !headFresh) ||
            (button.dataset.command === "image" && !screenImageReady);
    }
    if (fault) jogStatus.textContent = "Disabled · controller fault";
    else if (feedbackStale) jogStatus.textContent = "Disabled · stale robot state";
    else if (!jogEnabled.checked) jogStatus.textContent = "Off · monitoring only";
    else if (!fresh) jogStatus.textContent = "Paused · no fresh state";
    else if (busy) jogStatus.textContent = "Waiting for movement";
    else jogStatus.textContent = "On · ready for commands";
    if (connected && fault) {
        connectionStatus.textContent = "Controller fault";
        connectionDetail.textContent = "See the fault below";
        connectionDot.className = "dot error";
    } else if (connected && feedbackStale) {
        connectionStatus.textContent = "Robot feedback stale";
        connectionDetail.textContent = Number.isFinite(state.feedback_age_s) ? "Last feedback " + number(state.feedback_age_s, 1) + " s ago" : "Waiting for ROS updates";
        connectionDot.className = "dot error";
    } else if (connected && !fresh) {
        connectionStatus.textContent = "State connection stale";
        connectionDot.className = "dot error";
    }
}

function number(value, digits) {
    return typeof value === "number" && Number.isFinite(value) ? value.toFixed(digits) : "—";
}

function updateState(next) {
    state = next;
    connected = true;
    lastStateAt = Date.now();
    connectionStatus.textContent = "Connected";
    connectionDetail.textContent = "Received " + new Date().toLocaleTimeString();
    connectionDot.className = "dot live";
    document.getElementById("simulation-badge").hidden = !next.simulation;
    const warning = document.getElementById("robot-warning");
    warning.hidden = !next.fault && !next.feedback_stale;
    warning.textContent = next.fault ? "Controller fault: " + next.fault : next.feedback_stale ? "Robot feedback is stale. Controls are disabled until fresh feedback arrives." : "";
    const head = next.head;
    document.getElementById("head-pan").textContent = head && Number.isFinite(head.pan_rad) ?
        number(head.pan_rad * radiansToDegrees, 1) + "° / " + number(head.pan_rad, 3) + " rad" : "—";
    const headStatus = document.getElementById("head-status");
    headStatus.textContent = !head ? "Waiting for head state" : head.feedback_stale ? "Head feedback stale" :
        (head.panning ? "Panning" : "Pan idle") + " · " + (head.nodding ? "Nodding" : "Nod idle");
    headStatus.classList.toggle("moving", !!(head && !head.feedback_stale && (head.panning || head.nodding)));
    for (const limb of ["left", "right"]) {
        const view = views[limb];
        const moving = !!next.movement_in_progress[limb];
        view.status.textContent = next.feedback_stale ? "Stale" : moving ? "Moving" : "Idle";
        view.status.classList.toggle("moving", moving);
        for (const joint of jointNames) {
            const row = view.joints[joint];
            const angle = next.joint_angles_rad[limb][limb + "_" + joint] * radiansToDegrees;
            const torque = next.joint_efforts_Nm[limb][limb + "_" + joint];
            row.angleText.textContent = number(angle, 1);
            row.torqueText.textContent = number(torque, 2);
            for (const [bar, value, scale] of [[row.angleBar, angle, 180], [row.torqueBar, torque, 20]]) {
                const width = Number.isFinite(value) ? Math.min(Math.abs(value) / scale, 1) * 50 : 0;
                bar.style.left = (value < 0 ? 50 - width : 50) + "%";
                bar.style.width = width + "%";
            }
        }
        const pose = next.end_effector_poses[limb];
        for (const [index, axis] of ["x", "y", "z"].entries()) view.axes[axis].textContent = number(pose.position_m[index], 3) + " m";
        const [w, x, y, z] = pose.orientation_wijk;
        // Fixed-axis XYZ Euler readouts; jog rotations are applied in the base frame.
        const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
        const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        for (const [axis, angle] of [["roll", roll], ["pitch", pitch], ["yaw", yaw]]) view.axes[axis].textContent = number(angle * radiansToDegrees, 1) + "°";
        view.quaternion.textContent = "Quaternion [w, x, y, z]: " + pose.orientation_wijk.map(value => number(value, 3)).join(", ");
        const gripper = next.grippers[limb];
        view.gripperPosition.textContent = number(gripper.position_percent, 1) + "%";
        view.gripperForce.textContent = number(gripper.force_percent, 1) + "%";
        view.gripperPositionBar.style.width = Math.max(0, Math.min(100, gripper.position_percent || 0)) + "%";
        view.gripperForceBar.style.width = Math.max(0, Math.min(100, gripper.force_percent || 0)) + "%";
        view.gripperStatus.textContent = next.feedback_stale ? "Stale" : gripper.moving ? "Moving" : gripper.grasping ? "Grasping" : "Idle";
    }
    updateControls();
}

jogEnabled.addEventListener("change", () => {
    updateControls();
    showActivity(jogEnabled.checked ? "Controls enabled. Commands run one at a time." : "Controls are off. An active movement continues until completion or Stop all motion.");
});

arms.addEventListener("click", async event => {
    const button = event.target.closest(".jog-button, .pose-button");
    if (!button || button.disabled) return;
    updateControls();
    if (button.disabled) return;
    if (button.dataset.pose) {
        await sendCommand("move_to_" + button.dataset.pose, {limb_name: button.dataset.limb},
            button.dataset.limb + " arm to " + button.dataset.pose + " pose");
        return;
    }
    const {limb, kind, axis, direction} = button.dataset;
    const rotation = kind === "joint" || ["roll", "pitch", "yaw"].includes(axis);
    const input = document.getElementById(kind === "gripper" ? "gripper-step" : rotation ? "angle-step" : "position-step");
    const step = Number(input.value);
    if (!input.checkValidity() || !Number.isFinite(step) || step <= 0) {
        input.reportValidity();
        showActivity("Enter a step within the displayed input limits.", true);
        return;
    }
    const params = {limb_name: limb};
    let method;
    if (kind === "joint") {
        method = "jog_joint";
        params.joint_name = axis;
        params.delta_rad = Number(direction) * step / radiansToDegrees;
    } else if (kind === "endpoint") {
        method = "jog_endpoint";
        params.axis = axis;
        params.delta = Number(direction) * step / (rotation ? radiansToDegrees : 1000);
    } else {
        method = "jog_gripper";
        params.delta_percent = Number(direction) * step;
    }
    await sendCommand(method, params, limb + " " + (axis || "gripper") + " jog");
});

// Arm, head and display commands share one operation and the same stop-race handling.
async function sendCommand(method, params, description) {
    updateControls();
    if (!jogEnabled.checked || jogEnabled.disabled || commandPending || operationId !== null || state.commands_busy) return;
    commandPending = true;
    const version = ++commandVersion;
    updateControls();
    showActivity("Sending " + description + "…");
    try {
        const result = await rpc(method, params);
        if (version !== commandVersion) return;
        if (!result || !result.operation_id) throw new Error("No operation ID returned. Check robot state before sending another command.");
        operationId = result.operation_id;
        showActivity("Running " + description + "…");
    } catch (error) {
        if (version !== commandVersion) return;
        jogEnabled.checked = false;
        showActivity(error.message, true);
    } finally {
        if (version === commandVersion) commandPending = false;
        updateControls();
    }
}

const screenPreview = document.getElementById("screen-preview");
const screenContext = screenPreview.getContext("2d", {alpha: false});
screenContext.fillStyle = "black";
screenContext.fillRect(0, 0, 1024, 600);

document.getElementById("screen-file").addEventListener("change", async event => {
    const version = ++screenUploadVersion;
    const file = event.target.files[0];
    screenImageReady = false;
    updateControls();
    screenContext.fillStyle = "black";
    screenContext.fillRect(0, 0, 1024, 600);
    if (!file) {
        document.getElementById("screen-detail").textContent = "Choose an image to preview.";
        return;
    }
    const imageURL = URL.createObjectURL(file);
    try {
        const image = new Image();
        image.src = imageURL;
        await image.decode();
        if (version !== screenUploadVersion) return;
        const scale = Math.min(1024 / image.naturalWidth, 600 / image.naturalHeight);
        const width = Math.round(image.naturalWidth * scale);
        const height = Math.round(image.naturalHeight * scale);
        screenContext.drawImage(image, Math.floor((1024 - width) / 2), Math.floor((600 - height) / 2), width, height);
        screenImageReady = true;
        document.getElementById("screen-detail").textContent = file.name + " · fitted to 1024 × 600 on black";
    } catch (error) {
        if (version === screenUploadVersion) document.getElementById("screen-detail").textContent = "Could not read this image: " + error.message;
    } finally {
        URL.revokeObjectURL(imageURL);
        updateControls();
    }
});

document.getElementById("head-controls").addEventListener("click", async event => {
    const button = event.target.closest(".head-command");
    if (!button || button.disabled) return;
    updateControls();
    if (button.disabled) return;
    const command = button.dataset.command;
    const inputIds = command === "pan" ? ["head-target", "head-speed"] : command === "halo" ? ["halo-red", "halo-green"] : [];
    for (const id of inputIds) {
        const input = document.getElementById(id);
        if (!input.checkValidity() || !Number.isFinite(Number(input.value)) || input.value === "") {
            input.reportValidity();
            showActivity("Enter a value within the displayed input limits.", true);
            return;
        }
    }
    let method;
    let params;
    try {
        if (command === "pan") {
            method = "set_head_pan_rad";
            params = {angle_rad: Number(document.getElementById("head-target").value) / radiansToDegrees,
                speed_percent: Number(document.getElementById("head-speed").value)};
        } else if (command === "nod") {
            method = "nod_head";
            params = {times: 1};
        } else if (command === "halo") {
            method = "set_halo_led";
            params = {red_percent: Number(document.getElementById("halo-red").value),
                green_percent: Number(document.getElementById("halo-green").value)};
        } else if (command === "sonar") {
            method = "set_sonar_leds";
            params = {led_states: document.getElementById("sonar-mode").value};
        } else if (command === "color") {
            const color = document.getElementById("screen-color").value;
            method = "show_screen_color";
            params = {color_rgb: [1, 3, 5].map(start => parseInt(color.slice(start, start + 2), 16))};
        } else if (command === "image") {
            // Canvas gives RGBA. Send packed RGB as base64; no image package is needed.
            const rgba = screenContext.getImageData(0, 0, 1024, 600).data;
            const rgb = new Uint8Array(1024 * 600 * 3);
            for (let source = 0, target = 0; source < rgba.length; source += 4) {
                rgb[target++] = rgba[source];
                rgb[target++] = rgba[source + 1];
                rgb[target++] = rgba[source + 2];
            }
            const chunks = [];
            for (let start = 0; start < rgb.length; start += 32768) {
                chunks.push(String.fromCharCode(...rgb.subarray(start, start + 32768)));
            }
            method = "show_screen_image_rgb";
            params = {width: 1024, height: 600, rgb_base64: btoa(chunks.join(""))};
        } else return;
        await sendCommand(method, params, button.textContent.toLowerCase());
    } catch (error) {
        showActivity("Could not prepare the command: " + error.message, true);
    }
});

document.getElementById("stop-all").addEventListener("click", async () => {
    const version = ++commandVersion;
    commandPending = true;
    operationId = null;
    jogEnabled.checked = false;
    updateControls();
    showActivity("Requesting stop for active robot motion…");
    try {
        await rpc("abort_movement", {limb_name: null});
        if (version !== commandVersion) return;
        showActivity("Stop requested. Controls are off; monitor the live movement indicators.");
    } catch (error) {
        if (version !== commandVersion) return;
        showActivity("Stop request failed: " + error.message, true);
    } finally {
        if (version === commandVersion) commandPending = false;
        updateControls();
    }
});

// One state request at a time. Failed reads recover automatically; motion is never retried.
async function poll() {
    try {
        updateState(await rpc("get_state"));
    } catch (error) {
        connected = false;
        jogEnabled.checked = false;
        connectionStatus.textContent = "Disconnected";
        connectionDetail.textContent = "Retrying state connection";
        connectionDot.className = "dot error";
        showActivity(error.message, true);
        updateControls();
    }
    if (connected && operationId !== null) {
        const id = operationId;
        try {
            const operation = await rpc("get_operation", {operation_id: id});
            // A stop request can replace the operation while this read is in flight.
            if (operationId === id && ["succeeded", "failed", "cancelled"].includes(operation.status)) {
                operationId = null;
                if (operation.status === "failed") {
                    jogEnabled.checked = false;
                    const error = operation.error;
                    showActivity("Command failed: " + (typeof error === "string" ? error : error && error.message || "See the server log."), true);
                } else {
                    showActivity(operation.status === "cancelled" ? "Command cancelled." : "Command complete.");
                }
                updateControls();
            }
        } catch (error) {
            if (operationId === id) {
                operationId = null;
                jogEnabled.checked = false;
                showActivity("Could not read command status: " + error.message + " Check robot state before enabling controls again.", true);
                updateControls();
            }
        }
    }
    setTimeout(poll, 200);
}

setInterval(updateControls, 250);
poll();

// Each camera has its own request loop. A missing camera never interrupts control.
for (const [name, title] of [["left_hand_camera", "Left wrist"], ["right_hand_camera", "Right wrist"]]) {
    const card = element("article", "camera-card", undefined, document.getElementById("camera-grid"));
    const heading = element("div", "camera-title", undefined, card);
    element("strong", "", title, heading);
    const status = element("span", "camera-status", "Waiting", heading);
    const preview = element("div", "camera-preview", undefined, card);
    const image = element("img", "", undefined, preview);
    image.alt = title + " camera snapshot";
    image.hidden = true;
    const placeholder = element("span", "camera-placeholder", "Waiting for the first image…", preview);
    const detail = element("p", "camera-detail", name, card);
    let imageURL = null;

    async function refreshCamera() {
        if (document.hidden) {
            image.hidden = true;
            placeholder.hidden = false;
            placeholder.textContent = "Paused while this tab is hidden";
            status.textContent = "Paused";
            status.className = "camera-status";
            setTimeout(refreshCamera, 1000);
            return;
        }
        const abort = new AbortController();
        const timeout = setTimeout(() => abort.abort(), 4000);
        try {
            const response = await fetch("/camera/" + name + ".png", {
                cache: "no-store", signal: abort.signal
            });
            if (!response.ok) {
                const error = await response.json();
                throw new Error(error.message || "Camera unavailable (HTTP " + response.status + ")");
            }
            const blob = await response.blob();
            if (blob.type !== "image/png" || !blob.size) throw new Error("Camera returned an invalid image");
            if (imageURL) URL.revokeObjectURL(imageURL);
            imageURL = URL.createObjectURL(blob);
            image.src = imageURL;
            await image.decode();
            image.hidden = false;
            placeholder.hidden = true;
            status.textContent = "Received " + new Date().toLocaleTimeString();
            status.className = "camera-status live";
            detail.textContent = name;
        } catch (error) {
            image.hidden = true;
            image.removeAttribute("src");
            if (imageURL) URL.revokeObjectURL(imageURL);
            imageURL = null;
            placeholder.hidden = false;
            placeholder.textContent = "Camera unavailable";
            status.textContent = "Unavailable";
            status.className = "camera-status error";
            detail.textContent = error.name === "AbortError" ? "Image request timed out; retrying." : error.message;
        } finally {
            clearTimeout(timeout);
            // Schedule after completion so this camera never has overlapping requests.
            setTimeout(refreshCamera, 1000);
        }
    }
    refreshCamera();
}
