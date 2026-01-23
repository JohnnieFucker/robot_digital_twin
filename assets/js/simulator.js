import * as THREE from "./three.module.js";
import { OrbitControls } from "./OrbitControls.js";
import URDFLoader from "./URDFLoader.js";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

/**
 * 天工机器人仿真器类
 * 负责 3D 场景初始化、模型加载、关节控制及交互逻辑
 */
export class TiangongSimulator {
    constructor() {
        // Three.js 核心组件
        this.scene = null; // 场景
        this.camera = null; // 相机
        this.renderer = null; // 渲染器
        this.controls = null; // 轨道控制器 (OrbitControls)

        // 机器人模型相关
        this.robot = null; // 加载后的机器人模型对象
        this.joints = {}; // 存储机器人所有可动关节的引用
        this.jointControls = {}; // 存储 UI 控制滑块的引用
        this.overlays = new Map(); // 用于存储不同 Link 对应的 DOM 元素和配置
        this.weldingIndicators = new Map();
        this.robotJointGroups = { Link1: [], Link2: [] };
        this.state = {
            robots: {
                Link1: { name: "步兵1号", running: "在线", mode: "单机", welding: true, joints: {}, weldpoolUrl: "http://10.13.120.108:8889/weldpool/?autoplay=1&controls=0" },
                Link2: { name: "步兵2号", running: "在线", mode: "单机", welding: true, joints: {}, weldpoolUrl: "" }
            }
        };

        this.linkStatus = {
            el: null,
            target: null,
            targetLinkName: "Link1",
            worldOffset: new THREE.Vector3(0, 0, 3.5),
            lastContentUpdateMs: 0
        };

        // 状态标志
        this.isLoaded = false; // 模型是否加载完成

        // 动画与性能监控
        this.animationId = null;
        this.lastTime = 0;
        this.frameCount = 0;
        this.fps = 0;

        // --- 配置：默认视图中心点 ---
        this.defaultViewCenter = new THREE.Vector3(0, 0, 3);

        // --- 配置：默认相机位置（决定初始缩放） ---
        this.defaultCameraPosition = new THREE.Vector3(20, 20, 15);

        // 启动初始化
        this.init();
    }

    /**
     * 初始化流程
     */
    init() {
        this.setupScene(); // 1. 创建场景与坐标系
        this.setupCamera(); // 2. 配置相机
        this.setupRenderer(); // 3. 配置渲染器
        this.setupControls(); // 4. 配置控制器
        this.setupLights(); // 5. 添加灯光
        this.setupEventListeners(); // 6. 绑定事件
        this.loadRobot(); // 7. 加载 URDF 模型
        this.animate(); // 8. 开始渲染循环
    }

    /**
     * 配置 3D 场景
     */
    setupScene() {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x001a33); // 设置背景色为深蓝色

        this.coordinateFrame = new THREE.Group();
        this.scene.add(this.coordinateFrame);

        const gridHelper = new THREE.GridHelper(40, 40);
        gridHelper.rotation.x = Math.PI / 2;
        gridHelper.material.opacity = 0.25;
        gridHelper.material.transparent = true;
        this.coordinateFrame.add(gridHelper);
    }

    /**
     * 使用 CanvasTexture + Sprite 生成文字标签
     */
    createTextLabel(text, color) {
        const canvas = document.createElement("canvas");
        const context = canvas.getContext("2d");
        const size = 64;
        canvas.width = size;
        canvas.height = size;

        context.font = "bold 48px Arial";
        context.textAlign = "center";
        context.textBaseline = "middle";
        context.fillStyle = color;
        context.fillText(text, size / 2, size / 2);

        const texture = new THREE.CanvasTexture(canvas);
        const material = new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false
        });

        const sprite = new THREE.Sprite(material);
        sprite.scale.set(1, 1, 1);
        return sprite;
    }

    /**
     * 配置相机初始位置
     */
    setupCamera() {
        const container = document.getElementById("viewer-container");
        this.camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 1000);
        this.camera.up.set(0, 0, 1);
        this.camera.position.copy(this.defaultCameraPosition);
        this.camera.lookAt(this.defaultViewCenter);
    }

    /**
     * 配置 WebGL 渲染器
     */
    setupRenderer() {
        const container = document.getElementById("viewer-container");
        this.renderer = new THREE.WebGLRenderer({
            antialias: true,
            alpha: true
        });
        this.renderer.setSize(container.clientWidth, container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputColorSpace = THREE.SRGBColorSpace;

        container.appendChild(this.renderer.domElement);
    }

    /**
     * 配置控制器
     */
    setupControls() {
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.target.copy(this.defaultViewCenter);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.enableZoom = true;
        this.controls.enablePan = true;
        this.controls.maxDistance = 150;
        this.controls.minDistance = 5;
        this.controls.rotateSpeed = 1.0;
        this.controls.zoomSpeed = 1.2;
        this.controls.panSpeed = 1.0;
        this.controls.screenSpacePanning = false;

        this.controls.addEventListener("change", () => {
            this.updateCameraDebugInfo();
            this.userInteracted = true;
        });

        this.controls.update();
        this.userInteracted = false;
    }

    /**
     * 配置场景灯光
     */
    setupLights() {
        const ambientLight = new THREE.HemisphereLight(0xffffff, 0x444444, 1.0);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 1.5);
        directionalLight.position.set(10, 20, 10);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        directionalLight.shadow.camera.near = 0.5;
        directionalLight.shadow.camera.far = 500;
        this.scene.add(directionalLight);

        const fillLight = new THREE.DirectionalLight(0xffffff, 0.8);
        fillLight.position.set(-10, 10, -10);
        this.scene.add(fillLight);

        const topLight = new THREE.PointLight(0xffffff, 1.0);
        topLight.position.set(0, 50, 0);
        this.scene.add(topLight);
    }

    /**
     * 绑定 UI 事件
     */
    setupEventListeners() {
        window.addEventListener("resize", () => this.onWindowResize());
        window.addEventListener("keydown", (e) => {
            if (e.key === "1") {
                const ind = this.weldingIndicators.get("Link1-6");
                if (ind) this.setWeldingState("Link1-6", !ind.isWelding);
            } else if (e.key === "2") {
                const ind = this.weldingIndicators.get("Link2-6");
                if (ind) this.setWeldingState("Link2-6", !ind.isWelding);
            }
        });
    }

    updateCameraDebugInfo() {
        //         const pos = this.camera.position;
        //         const target = this.controls.target;
        //         const quat = this.camera.quaternion;
        //         console.log(`相机配置代码:
        // this.camera.position.set(${pos.x.toFixed(2)}, ${pos.y.toFixed(2)}, ${pos.z.toFixed(2)});
        // this.controls.target.set(${target.x.toFixed(2)}, ${target.y.toFixed(2)}, ${target.z.toFixed(2)});
        // this.camera.quaternion.set(${quat.x.toFixed(3)}, ${quat.y.toFixed(3)}, ${quat.z.toFixed(3)}, ${quat.w.toFixed(3)});`);
    }

    async loadRobot() {
        try {
            const loader = new URDFLoader();
            loader.packages = { robot: "./" };
            this.showLoading(true);

            const robot = await new Promise((resolve, reject) => {
                loader.load(`./tiangong.urdf?t=${Date.now()}`, resolve, undefined, reject);
            });

            this.robot = robot;
            robot.position.set(14, 0, 0);
            robot.rotation.z = -Math.PI / 2;
            this.coordinateFrame.add(robot);
            robot.updateMatrixWorld(true);

            robot.traverse((child) => {
                if (child.isMesh) {
                    child.castShadow = true;
                    child.receiveShadow = true;
                }
            });

            this.setupJointInterface();
            this.setupBaseReferenceLines();
            this.isLoaded = true;
            this.showLoading(false);
            const s1 = this.state.robots.Link1;
            const s2 = this.state.robots.Link2;
            this.setLinkOverlay("Link1", "步兵1号", { weldpoolUrl: s1.weldpoolUrl }, new THREE.Vector3(0, 0, 3.8));
            this.setLinkOverlay("Link2", "步兵2号", { weldpoolUrl: s2.weldpoolUrl }, new THREE.Vector3(0, 0, 1.5));
            this.addWeldingIndicator("Link1-6", new THREE.Vector3(-0.45, 0.05, 0), 0.04, 800);
            this.addWeldingIndicator("Link2-6", new THREE.Vector3(-0.45, 0.05, 0), 0.04, 800);

            // 初始应用一遍状态，确保 Overlay 获取到所有数据
            this.applyState();

            this.setWeldingState("Link1-6", true);
            this.setWeldingState("Link2-6", true);
        } catch (error) {
            console.error("加载机器人失败:", error);
            this.showError("加载机器人模型失败: " + error.message);
            this.showLoading(false);
        }
    }

    setupJointInterface() {
        const container = document.getElementById("joints-container");
        container.innerHTML = "";
        this.robotJointGroups = { Link1: [], Link2: [] };

        const joints = Object.values(this.robot.joints).filter((j) => j.jointType !== "fixed");
        joints.sort((a, b) => a.name.localeCompare(b.name));

        const group = document.createElement("div");
        group.className = "joint-group";
        group.innerHTML = "<h3>关节控制</h3>";

        joints.forEach((joint) => {
            this.joints[joint.name] = joint;
            const valInit = joint.jointValue[0] || 0;
            if (joint.name.startsWith("joint1")) {
                this.robotJointGroups.Link1.push(joint.name);
                this.state.robots.Link1.joints[joint.name] = valInit;
            } else if (joint.name.startsWith("joint2")) {
                this.robotJointGroups.Link2.push(joint.name);
                this.state.robots.Link2.joints[joint.name] = valInit;
            }

            const control = document.createElement("div");
            control.className = "joint-control";
            const min = joint.limit.lower;
            const max = joint.limit.upper;
            const step = 0.01;

            control.innerHTML = `
                <h4>${joint.name}</h4>
                <div class="slider-container">
                    <input type="range" class="slider" 
                        min="${min}" max="${max}" step="${step}" 
                        value="${valInit}">
                    <div class="value-display">${valInit.toFixed(2)}</div>
                </div>
            `;

            const slider = control.querySelector(".slider");
            const display = control.querySelector(".value-display");

            slider.addEventListener("input", (e) => {
                const val = parseFloat(e.target.value);
                this.setJointValue(joint.name, val);
                display.textContent = val.toFixed(2);
                // 同步更新 state
                if (joint.name.startsWith("joint1")) {
                    this.state.robots.Link1.joints[joint.name] = val;
                } else if (joint.name.startsWith("joint2")) {
                    this.state.robots.Link2.joints[joint.name] = val;
                }
            });

            this.jointControls[joint.name] = { slider, display };
            group.appendChild(control);
        });

        container.appendChild(group);
        this.applyState();
    }

    setupBaseReferenceLines() {
        const baseLink = this.robot.links["base_link"];
        if (!baseLink) return;

        const refGroup = new THREE.Group();
        refGroup.name = "Base_Reference_Lines";
        baseLink.add(refGroup);

        const maxLength = 28;
        const step = 0.5;
        let tickSize = 0.2;

        const lineGeometry = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(-1.2, 0, 0), new THREE.Vector3(-1.2, -maxLength, 0)]);
        const lineMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00, linewidth: 2 });
        const mainLine = new THREE.Line(lineGeometry, lineMaterial);
        refGroup.add(mainLine);

        for (let d = 0; d <= maxLength; d += step) {
            const isMajor = (d * 10) % 2 === 0;
            tickSize = isMajor ? 0.4 : 0.2;
            const tickGeom = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(-1.2, -d, 0), new THREE.Vector3(-tickSize - 1.2, -d, 0)]);
            const tick = new THREE.Line(tickGeom, lineMaterial);
            refGroup.add(tick);

            const label = this.createTextLabel(isMajor ? d : d.toFixed(1), "#ffffff");
            label.position.set(-1.8, -d, 0);
            label.scale.set(isMajor ? 0.3 : 0.2, isMajor ? 0.3 : 0.2, isMajor ? 0.3 : 0.2);
            refGroup.add(label);
        }
    }

    setJointValue(name, value) {
        if (this.joints[name]) {
            this.joints[name].setJointValue(value);
        }
    }

    setLinkOverlay(linkName, robotName, data = {}, offset = new THREE.Vector3(0, 0, 2)) {
        if (!this.robot) return;
        const target = this.robot.links[linkName] || this.robot.getObjectByName(linkName);
        if (!target) return;

        if (!this.overlays.has(robotName)) {
            const el = document.createElement("div");
            el.className = "link-status-overlay";
            document.getElementById("viewer-container").appendChild(el);

            this.overlays.set(robotName, {
                el,
                target,
                offset,
                data,
                lastUpdate: 0
            });
        } else {
            const entry = this.overlays.get(robotName);
            entry.data = { ...entry.data, ...data };
        }
    }

    applyState() {
        const s1 = this.state.robots.Link1;
        const s2 = this.state.robots.Link2;

        const updateRobot = (robotState) => {
            Object.entries(robotState.joints).forEach(([name, val]) => {
                if (this.joints[name]) {
                    this.joints[name].setJointValue(val);
                    const ctrl = this.jointControls[name];
                    if (ctrl) {
                        ctrl.slider.value = val;
                        ctrl.display.textContent = Number(val).toFixed(2);
                    }
                }
            });
        };

        updateRobot(s1);
        updateRobot(s2);

        this.setWeldingState("Link1-6", !!s1.welding);
        this.setWeldingState("Link2-6", !!s2.welding);

        const updateOverlay = (robotName, robotState) => {
            const ov = this.overlays.get(robotName);
            if (ov) {
                const newData = { 运行: robotState.running, 模式: robotState.mode, 焊接: robotState.welding ? "焊接中" : "等待中", weldpoolUrl: robotState.weldpoolUrl };
                ov.data = { ...ov.data, ...newData };
            }
        };

        updateOverlay("步兵1号", s1);
        updateOverlay("步兵2号", s2);
    }

    updateAllOverlays() {
        if (!this.camera || this.overlays.size === 0) return;
        const container = document.getElementById("viewer-container");
        const rect = container.getBoundingClientRect();
        const now = performance.now();

        this.overlays.forEach((info, robotName) => {
            const { el, target, offset, data } = info;

            if (now - info.lastUpdate > 1000) {
                this.renderOverlayHTML(el, robotName, data);
                info.lastUpdate = now;
            }

            const worldPos = new THREE.Vector3();
            target.getWorldPosition(worldPos);
            worldPos.add(offset);

            const ndc = worldPos.clone().project(this.camera);
            if (ndc.z < -1 || ndc.z > 1) {
                el.style.display = "none";
                return;
            }

            const x = (ndc.x * 0.5 + 0.5) * rect.width;
            const y = (-ndc.y * 0.5 + 0.5) * rect.height;

            el.style.display = "block";
            el.style.transform = `translate(${x}px, ${y}px) translate(-50%, -110%)`;
        });
    }

    addWeldingIndicator(linkName, localOffset = new THREE.Vector3(0, 0, 0), size = 0.08, blinkMs = 400) {
        if (!this.robot) return;
        const target = this.robot.links[linkName] || this.robot.getObjectByName(linkName);
        if (!target) return;

        const geometry = new THREE.SphereGeometry(size, 16, 16);
        const material = new THREE.MeshBasicMaterial({ color: 0xff0000, transparent: true });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.copy(localOffset);
        target.add(mesh);
        this.weldingIndicators.set(linkName, { target, mesh, blinkMs, isWelding: false });
    }

    setWeldingState(linkName, isWelding) {
        const ind = this.weldingIndicators.get(linkName);
        if (ind) ind.isWelding = !!isWelding;
    }

    updateWeldingIndicators(now) {
        this.weldingIndicators.forEach((ind) => {
            if (ind.mesh) {
                ind.mesh.visible = ind.isWelding ? Math.floor(now / ind.blinkMs) % 2 === 0 : false;
            }
        });
    }

    renderOverlayHTML(el, title, data) {
        const timeStr = new Date().toLocaleTimeString("zh-CN", { hour12: false });
        const { weldpoolUrl, ...otherData } = data;

        // 构造属性行 HTML
        const rowsHtml = Object.entries(otherData)
            .map(([key, val]) => `<div class="link-status-row"><span class="link-status-key">${key}</span><span class="link-status-val">${val}</span></div>`)
            .join("");

        // 如果是第一次渲染或结构不匹配，初始化基础结构
        if (!el.querySelector(".link-status-content")) {
            el.innerHTML = `
                <div class="link-status-title">${title}</div>
                <div class="link-status-content">${rowsHtml}</div>
                <div class="link-status-footer">
                    <div class="link-status-row">
                        <span class="link-status-key">刷新</span>
                        <span class="link-status-val refresh-time">${timeStr}</span>
                    </div>
                </div>
                <div class="link-status-monitor-container"></div>
            `;
        } else {
            // 更新属性行（仅内容变化时）
            const contentEl = el.querySelector(".link-status-content");
            if (contentEl.innerHTML !== rowsHtml) {
                contentEl.innerHTML = rowsHtml;
            }
            // 更新时间
            const timeEl = el.querySelector(".refresh-time");
            if (timeEl) timeEl.textContent = timeStr;
        }

        // 更新监控画面 (仅在 URL 变化时操作，避免 iframe 重载)
        const monitorContainer = el.querySelector(".link-status-monitor-container");
        if (weldpoolUrl) {
            let iframe = monitorContainer.querySelector(".monitor-mini-iframe");
            if (!iframe) {
                monitorContainer.innerHTML = `
                    <div class="link-status-monitor">
                        <div class="monitor-label">实时熔池</div>
                        <iframe src="${weldpoolUrl}" class="monitor-mini-iframe"></iframe>
                    </div>
                `;
            } else if (iframe.getAttribute("src") !== weldpoolUrl) {
                iframe.src = weldpoolUrl;
            }
        } else {
            monitorContainer.innerHTML = "";
        }
    }

    animate() {
        requestAnimationFrame(() => this.animate());
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
            this.controls.update();
        }
        const now = performance.now();
        this.updateWeldingIndicators(now);
        this.updateAllOverlays();
    }

    onWindowResize() {
        const container = document.getElementById("viewer-container");
        if (!container) return;
        const width = container.clientWidth;
        const height = container.clientHeight;
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    showLoading(show) {
        const el = document.getElementById("loading");
        if (el) el.style.display = show ? "block" : "none";
    }

    showError(msg) {
        const el = document.getElementById("error");
        if (el) {
            el.textContent = msg;
            el.style.display = "block";
            setTimeout(() => {
                el.style.display = "none";
            }, 5000);
        }
    }
}
