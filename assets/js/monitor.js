export class MonitorManager {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.monitors = [
            { id: 1, title: "现场监控", url: "https://dataease.cgboiler.com/#/de-link/LVfVeshS" },
            { id: 2, title: "现场监控", url: "https://pv-monitor.cgboiler.com/?y1=93f4f02557984e9fa6ac4e60df96adcd" }
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
            <div class="monitor-wrapper">
                <div class="monitor-card" 
                     data-id="${monitor.id}"
                     onclick="this.dataset.expanded = this.dataset.expanded === 'true' ? 'false' : 'true'">
                    <div class="monitor-header">
                        <div class="monitor-title">${monitor.title} ${monitor.id}</div>
                    </div>
                    <div class="monitor-body">
                        ${monitor.url ? `<iframe class="monitor-iframe" src="${monitor.url}" allow="autoplay; fullscreen"></iframe>` : `<div class="monitor-placeholder">等待信号接入...</div>`}
                        <div class="monitor-click-overlay"></div>
                    </div>
                </div>
            </div>
        `
            )
            .join("");
    }
}
