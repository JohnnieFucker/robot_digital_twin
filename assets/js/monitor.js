export class MonitorManager {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.monitors = [
            { id: 1, title: "现场监控1", url: "https://pv-monitor.cgboiler.com/?y1=rtsp://admin:center123@10.13.75.77/cam/realmonitor?channel=1&subtype=1" },
            { id: 2, title: "现场监控2", url: "" },
            { id: 3, title: "现场监控3", url: "" }
        ];
    }

    init() {
        if (!this.container) return;
        this.render();
    }

    render() {
        this.container.innerHTML = this.monitors
            .map(
                (monitor) => `
            <div class="monitor-card bg-slate-800/50 border border-slate-700 rounded-lg overflow-hidden group cursor-pointer transition-all duration-300" 
                 data-id="${monitor.id}"
                 onclick="this.dataset.expanded = this.dataset.expanded === 'true' ? 'false' : 'true'">
                <div class="monitor-header">
                    <div class="monitor-title">${monitor.title}</div>
                </div>
                <div class="monitor-body">
                    ${monitor.url ? `<iframe class="monitor-iframe" src="${monitor.url}" allow="autoplay; fullscreen"></iframe>` : `<div class="monitor-placeholder">等待信号接入...</div>`}
                    <div class="monitor-click-overlay"></div>
                </div>
            </div>
        `
            )
            .join("");

        // 添加点击放大缩小的样式支持
        if (!document.getElementById("monitor-style")) {
            const style = document.createElement("style");
            style.id = "monitor-style";
            style.textContent = `
                .monitor-card[data-expanded="true"] {
                    position: fixed;
                    top: 50%;
                    left: 50%;
                    transform: translate(-50%, -50%);
                    width: 80vw;
                    z-index: 2000;
                    box-shadow: 0 0 50px rgba(0,0,0,0.8);
                }
                .monitor-overlay-container:has(.monitor-card[data-expanded="true"]) {
                    z-index: 2000;
                }
                .monitor-card[data-expanded="true"] .aspect-video {
                    aspect-ratio: 16 / 9;
                }
                .monitor-click-overlay {
                    position: absolute;
                    top: 0;
                    left: 0;
                    width: 100%;
                    height: 100%;
                    z-index: 10;
                    cursor: pointer;
                }
            `;
            document.head.appendChild(style);
        }
    }
}
