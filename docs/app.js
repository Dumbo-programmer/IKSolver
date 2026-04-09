(() => {
  const yearEl = document.getElementById("year");
  if (yearEl) {
    yearEl.textContent = String(new Date().getFullYear());
  }

  const reveals = document.querySelectorAll(".reveal");
  const observer = new IntersectionObserver(
    (entries) => {
      for (const entry of entries) {
        if (entry.isIntersecting) {
          entry.target.classList.add("visible");
          observer.unobserve(entry.target);
        }
      }
    },
    { threshold: 0.14 }
  );
  reveals.forEach((node) => observer.observe(node));

  const canvas = document.getElementById("ik-canvas");
  if (!canvas) {
    return;
  }

  const ctx = canvas.getContext("2d");
  const dpr = Math.max(1, window.devicePixelRatio || 1);
  let width = 960;
  let height = 520;

  const state = {
    algorithm: "ccd",
    base: { x: 0, y: 0 },
    lengths: [120, 95, 75, 52],
    joints: [],
    target: { x: 0, y: 0 }
  };

  function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    width = Math.max(320, Math.floor(rect.width));
    height = Math.max(260, Math.floor(rect.height));

    canvas.width = Math.floor(width * dpr);
    canvas.height = Math.floor(height * dpr);

    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    state.base.x = Math.round(width * 0.28);
    state.base.y = Math.round(height * 0.55);

    if (!state.target.initialized) {
      state.target.x = Math.round(width * 0.72);
      state.target.y = Math.round(height * 0.45);
      state.target.initialized = true;
    }

    resetChain();
  }

  function resetChain() {
    state.joints = [{ x: state.base.x, y: state.base.y }];
    for (let i = 0; i < state.lengths.length; i += 1) {
      const prev = state.joints[i];
      state.joints.push({ x: prev.x + state.lengths[i], y: prev.y });
    }
  }

  function dist(a, b) {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return Math.hypot(dx, dy);
  }

  function solveCCD(iterations) {
    for (let iter = 0; iter < iterations; iter += 1) {
      for (let i = state.joints.length - 2; i >= 0; i -= 1) {
        const joint = state.joints[i];
        const end = state.joints[state.joints.length - 1];

        const toEndX = end.x - joint.x;
        const toEndY = end.y - joint.y;
        const toTargetX = state.target.x - joint.x;
        const toTargetY = state.target.y - joint.y;

        const angleA = Math.atan2(toEndY, toEndX);
        const angleB = Math.atan2(toTargetY, toTargetX);
        let delta = angleB - angleA;

        while (delta > Math.PI) delta -= Math.PI * 2;
        while (delta < -Math.PI) delta += Math.PI * 2;

        const maxStep = 0.33;
        if (Math.abs(delta) > maxStep) {
          delta = Math.sign(delta) * maxStep;
        }

        const c = Math.cos(delta);
        const s = Math.sin(delta);

        for (let j = i + 1; j < state.joints.length; j += 1) {
          const relX = state.joints[j].x - joint.x;
          const relY = state.joints[j].y - joint.y;
          state.joints[j].x = joint.x + relX * c - relY * s;
          state.joints[j].y = joint.y + relX * s + relY * c;
        }
      }
    }
  }

  function solveFABRIK(iterations) {
    const joints = state.joints;
    const n = joints.length;
    const totalLength = state.lengths.reduce((sum, len) => sum + len, 0);
    const base = state.base;

    if (dist(base, state.target) >= totalLength) {
      const dx = state.target.x - base.x;
      const dy = state.target.y - base.y;
      const mag = Math.hypot(dx, dy) || 1;
      const ux = dx / mag;
      const uy = dy / mag;
      joints[0].x = base.x;
      joints[0].y = base.y;

      for (let i = 1; i < n; i += 1) {
        joints[i].x = joints[i - 1].x + ux * state.lengths[i - 1];
        joints[i].y = joints[i - 1].y + uy * state.lengths[i - 1];
      }
      return;
    }

    for (let iter = 0; iter < iterations; iter += 1) {
      joints[n - 1].x = state.target.x;
      joints[n - 1].y = state.target.y;

      for (let i = n - 2; i >= 0; i -= 1) {
        const dx = joints[i].x - joints[i + 1].x;
        const dy = joints[i].y - joints[i + 1].y;
        const mag = Math.hypot(dx, dy) || 1;
        const ratio = state.lengths[i] / mag;

        joints[i].x = joints[i + 1].x + dx * ratio;
        joints[i].y = joints[i + 1].y + dy * ratio;
      }

      joints[0].x = base.x;
      joints[0].y = base.y;

      for (let i = 0; i < n - 1; i += 1) {
        const dx = joints[i + 1].x - joints[i].x;
        const dy = joints[i + 1].y - joints[i].y;
        const mag = Math.hypot(dx, dy) || 1;
        const ratio = state.lengths[i] / mag;

        joints[i + 1].x = joints[i].x + dx * ratio;
        joints[i + 1].y = joints[i].y + dy * ratio;
      }
    }
  }

  function drawGrid() {
    ctx.save();
    ctx.strokeStyle = "rgba(116, 158, 176, 0.17)";
    ctx.lineWidth = 1;
    const step = 34;

    for (let x = 0; x <= width; x += step) {
      ctx.beginPath();
      ctx.moveTo(x + 0.5, 0);
      ctx.lineTo(x + 0.5, height);
      ctx.stroke();
    }

    for (let y = 0; y <= height; y += step) {
      ctx.beginPath();
      ctx.moveTo(0, y + 0.5);
      ctx.lineTo(width, y + 0.5);
      ctx.stroke();
    }

    ctx.restore();
  }

  function drawScene() {
    ctx.clearRect(0, 0, width, height);
    drawGrid();

    const joints = state.joints;

    ctx.beginPath();
    ctx.lineWidth = 5;
    ctx.lineCap = "round";
    ctx.strokeStyle = "#50d0b4";
    ctx.moveTo(joints[0].x, joints[0].y);
    for (let i = 1; i < joints.length; i += 1) {
      ctx.lineTo(joints[i].x, joints[i].y);
    }
    ctx.stroke();

    for (let i = 0; i < joints.length; i += 1) {
      const r = i === joints.length - 1 ? 6 : 7;
      ctx.beginPath();
      ctx.fillStyle = "#ecf3f7";
      ctx.arc(joints[i].x, joints[i].y, r, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "rgba(8, 14, 18, 0.85)";
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    ctx.beginPath();
    ctx.arc(state.target.x, state.target.y, 8, 0, Math.PI * 2);
    ctx.fillStyle = "#ff6e52";
    ctx.fill();

    ctx.beginPath();
    ctx.moveTo(state.target.x - 12, state.target.y);
    ctx.lineTo(state.target.x + 12, state.target.y);
    ctx.moveTo(state.target.x, state.target.y - 12);
    ctx.lineTo(state.target.x, state.target.y + 12);
    ctx.strokeStyle = "rgba(255, 122, 95, 0.95)";
    ctx.lineWidth = 2;
    ctx.stroke();

    const end = joints[joints.length - 1];
    const e = dist(end, state.target);
    ctx.fillStyle = "#b8d2dd";
    ctx.font = '13px "IBM Plex Mono", monospace';
    ctx.fillText(`algorithm: ${state.algorithm.toUpperCase()} | error: ${e.toFixed(2)} px`, 16, 24);
  }

  function animate() {
    if (state.algorithm === "ccd") {
      solveCCD(4);
    } else {
      solveFABRIK(3);
    }

    drawScene();
    requestAnimationFrame(animate);
  }

  function updateTargetFromEvent(event) {
    const rect = canvas.getBoundingClientRect();
    let clientX;
    let clientY;

    if (event.touches && event.touches.length > 0) {
      clientX = event.touches[0].clientX;
      clientY = event.touches[0].clientY;
    } else {
      clientX = event.clientX;
      clientY = event.clientY;
    }

    state.target.x = Math.min(width - 16, Math.max(16, clientX - rect.left));
    state.target.y = Math.min(height - 16, Math.max(16, clientY - rect.top));
  }

  canvas.addEventListener("mousemove", updateTargetFromEvent);
  canvas.addEventListener("touchstart", (event) => {
    updateTargetFromEvent(event);
  }, { passive: true });
  canvas.addEventListener("touchmove", (event) => {
    updateTargetFromEvent(event);
  }, { passive: true });

  const buttons = document.querySelectorAll(".algo-btn");
  buttons.forEach((button) => {
    button.addEventListener("click", () => {
      buttons.forEach((b) => b.classList.remove("active"));
      button.classList.add("active");
      state.algorithm = button.dataset.algo || "ccd";
      resetChain();
    });
  });

  window.addEventListener("resize", resizeCanvas);

  resizeCanvas();
  animate();
})();
