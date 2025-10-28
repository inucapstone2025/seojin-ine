// viewer.js — RunSync web viewer (single capture + 360° scan, point-size fix)

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { PLYLoader } from 'three/addons/loaders/PLYLoader.js';

(() => {
  // ---------- DOM refs ----------
  const btnSingle   = document.getElementById('btn');       // 단일 촬영
  const btn360      = document.getElementById('btn360');    // 360° 스캔
  const statusEl    = document.getElementById('status') || { textContent: '' };
  const metaEl      = document.getElementById('meta')   || { innerHTML: '' };
  const nameInput   = document.getElementById('name')   || { value: 'anon' };
  const genderInput = document.getElementById('gender') || { value: 'U' };
  const mount       = document.getElementById('viewer') || document.getElementById('three-canvas') || document.body;

  // ---------- Three.js core ----------
  let renderer, scene, camera, controls, currentObject;

  function initThree() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x101014);

    const w = Math.max(1, mount.clientWidth || window.innerWidth);
    const h = Math.max(1, mount.clientHeight || window.innerHeight);

    camera = new THREE.PerspectiveCamera(50, w / h, 0.001, 100);
    camera.position.set(0.2, 0.2, 0.6);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(w, h);
    mount.appendChild(renderer.domElement);

    // Lights
    const hemi = new THREE.HemisphereLight(0xffffff, 0x202030, 0.8);
    scene.add(hemi);
    const dir = new THREE.DirectionalLight(0xffffff, 0.8);
    dir.position.set(1, 1, 1);
    scene.add(dir);

    // Ground grid (optional)
    const grid = new THREE.GridHelper(1.0, 20, 0x333355, 0x222233);
    grid.position.y = -0.05;
    scene.add(grid);

    // Controls
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.rotateSpeed = 0.6;
    controls.zoomSpeed = 1.0;
    controls.panSpeed = 0.8;

    window.addEventListener('resize', onResize);
    animate();
  }

  function onResize() {
    const w = Math.max(1, mount.clientWidth || window.innerWidth);
    const h = Math.max(1, mount.clientHeight || window.innerHeight);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }

  function animate() {
    requestAnimationFrame(animate);
    controls && controls.update();
    renderer.render(scene, camera);
  }

  // ---------- Helpers ----------
  function setStatus(msg) {
    statusEl.textContent = msg;
    console.log('[status]', msg);
  }

  function disposeObject(obj) {
    if (!obj) return;
    obj.traverse?.(child => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) {
        if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
        else child.material.dispose();
      }
    });
    scene.remove(obj);
  }

  // 안정적인 프레이밍 (bounding box 기반)
  function fitCameraToObject(object, offset = 1.35) {
    const box = new THREE.Box3().setFromObject(object);
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    box.getSize(size);
    box.getCenter(center);

    const maxDim = Math.max(size.x, size.y, size.z) || 0.1;
    const fov = camera.fov * (Math.PI / 180);
    const dist = Math.abs((maxDim / 2) / Math.tan(fov / 2)) * offset;

    // +Z 방향에서 바라보도록 배치
    const dir = new THREE.Vector3(0, 0, 1);
    const newPos = center.clone().add(dir.multiplyScalar(dist));

    camera.position.copy(newPos);
    camera.near = Math.max(dist / 100, 0.001);
    camera.far  = dist * 100;
    camera.updateProjectionMatrix();

    controls.target.copy(center);
    controls.maxDistance = dist * 10;
    controls.update();
  }

  // ---------- PLY Loader ----------
  const plyLoader = new PLYLoader();

  async function loadPLY(url) {
    return new Promise((resolve, reject) => {
      setStatus('PLY 로드 중…');
      plyLoader.load(
        url,
        geometry => {
          geometry.computeBoundingBox?.();
          geometry.computeBoundingSphere?.();

          const hasColor = !!(geometry.attributes?.color && geometry.attributes.color.itemSize >= 3);

          // 거리 감쇠 비활성화로 점 크기 고정 → 멀어져도 작아지지 않음
          const material = new THREE.PointsMaterial({
            size: 2.0,                 // px 단위, 1.5~3.0 사이 취향대로
            sizeAttenuation: false,    // 핵심!
            vertexColors: hasColor,
            color: hasColor ? undefined : 0xcccccc
          });

          const obj = new THREE.Points(geometry, material);

          disposeObject(currentObject);
          currentObject = obj;
          scene.add(currentObject);

          fitCameraToObject(currentObject, 1.4);
          setStatus('미리보기 표시됨');
          resolve(obj);
        },
        xhr => {
          const total = xhr.total || 0;
          if (total > 0) {
            const p = ((xhr.loaded / total) * 100).toFixed(1);
            setStatus(`PLY 로드 중… ${p}%`);
          }
        },
        err => {
          console.error(err);
          setStatus('PLY 로드 실패');
          reject(err);
        }
      );
    });
  }

  // ---------- API calls ----------
    async function apiStartCapture() {
        const name = nameInput.value || 'anon';
        const gender = genderInput.value || 'U';
        btnSingle && (btnSingle.disabled = true);
        setStatus('단일 촬영 요청 중… (Pi 캡처)');

        try {
            const resp = await fetch('/start-capture', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ name, gender })
            });

            if (!resp.ok) {
                const text = await resp.text();
                throw new Error(text);
            }

            setStatus('촬영 요청 전송 완료');
            // 여기서 PLY 로드 관련 코드는 제거
            // await loadPLY(data.modelUrl);
        } catch (e) {
            console.error(e);
            setStatus('에러: ' + e.message);
        } finally {
            btnSingle && (btnSingle.disabled = false);
        }
    }


  async function apiScan360() {
    const name = nameInput.value || 'anon';
    const gender = genderInput.value || 'U';
    btn360 && (btn360.disabled = true);
    setStatus('360° 스캔 중… (모터 회전 + 8장 Pi 캡처 + 정합)');
    try {
      const resp = await fetch('/scan-360', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name, gender })
      });
      const text = await resp.text();
      if (!resp.ok) {
        let msg = text;
        try { msg = JSON.parse(text).error || text; } catch {}
        throw new Error(msg);
      }
      const data = JSON.parse(text);
      if (!data?.finalUrl) throw new Error('finalUrl이 응답에 없습니다.');
      setStatus('최종 PLY 로드 중…');
      await loadPLY(data.finalUrl);
      metaEl.innerHTML = `
        <div>세션: <a href="/captures/${(data.session||'').split('/').pop()}" target="_blank">${data.session||'-'}</a></div>
        <div>최종 PLY: <a href="${data.finalUrl}" target="_blank">${data.finalUrl}</a></div>
        <div>촬영 각도: ${Array.isArray(data.angles) ? data.angles.join(', ') : '-'}</div>
      `;
    } catch (e) {
      console.error(e);
      setStatus('에러: ' + e.message);
    } finally {
      btn360 && (btn360.disabled = false);
    }
  }

  // ---------- Bind events ----------
  function bindUI() {
    btnSingle && btnSingle.addEventListener('click', apiStartCapture);
    btn360   && btn360.addEventListener('click', apiScan360);
  }

  // ---------- Kickoff ----------
  initThree();
  bindUI();

  // 개발 편의: URL 쿼리로 임의 PLY 미리보기
  // 예: http://<host>:9000/?ply=/captures/2025.../aligned/final_aligned.ply
  try {
    const params = new URLSearchParams(window.location.search);
    const ply = params.get('ply');
    if (ply) loadPLY(ply);
  } catch {}
})();
