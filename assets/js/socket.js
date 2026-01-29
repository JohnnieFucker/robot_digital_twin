/**
 * SocketManager - 基于 Paho MQTT 的连接管理器
 */
export class SocketManager {
    constructor(config, onMessage) {
        this.config = {
            hostname: config.hostname || "mqtt.cgboiler.com",
            port: config.port || 443,
            clientId: config.clientId || "webClient_" + parseInt(Math.random() * 1000, 10),
            topic: config.topic || "cgRobot_tg_msb",
            path: config.path || "/mqtt",
            userName: config.userName || "cgIOT",
            password: config.password || "!Znhzx2024",
            useSSL: config.useSSL !== undefined ? config.useSSL : true
        };

        this.onMessage = onMessage;
        this.client = null;
        this.reconnectAttempts = 0;
        this.reconnectInterval = 5000;

        this.init();
    }

    init() {
        try {
            // 初始化 Paho MQTT 客户端
            // 使用更长的随机 ID 以防冲突
            const clientId = this.config.clientId + "_" + Math.random().toString(16).slice(2, 8);
            this.client = new Paho.MQTT.Client(this.config.hostname, Number(this.config.port), this.config.path, clientId);

            // 设置回调
            this.client.onConnectionLost = this.onConnectionLost.bind(this);
            this.client.onMessageArrived = this.onMessageArrived.bind(this);

            this.connect();
        } catch (error) {
            console.error("MQTT 客户端初始化失败:", error);
        }
    }

    connect() {
        const connectOptions = {
            onSuccess: this.onConnect.bind(this),
            onFailure: this.onConnectFailure.bind(this),
            useSSL: this.config.useSSL,
            timeout: 10,
            userName: this.config.userName,
            password: this.config.password,
            keepAliveInterval: 120, // 降低心跳间隔
            cleanSession: true,
            mqttVersion: 4 // 明确指定 MQTT 3.1.1 (Level 4)
        };

        console.log(`正在连接到 MQTT 服务器: ${this.config.hostname}:${this.config.port}...`);
        try {
            this.client.connect(connectOptions);
        } catch (e) {
            console.error("MQTT 连接触发异常:", e);
            this.handleReconnect();
        }
    }

    onConnect() {
        console.log("%c MQTT 成功连接 ", "background: #222; color: #bada55");
        this.reconnectAttempts = 0;

        // 连接成功后订阅主题
        this.client.subscribe(this.config.topic);
        console.log(`已订阅主题: ${this.config.topic}`);
    }

    onConnectFailure(response) {
        console.error("MQTT 连接失败: " + response.errorMessage);
        this.handleReconnect();
    }

    onConnectionLost(response) {
        if (response.errorCode !== 0) {
            console.warn("MQTT 连接丢失: " + response.errorMessage);
            this.handleReconnect();
        }
    }

    onMessageArrived(message) {
        // console.log("收到 MQTT 消息: " + message.payloadString);
        if (this.onMessage) {
            try {
                // 尝试解析 JSON，如果不是 JSON 则直接传递字符串
                let data;
                try {
                    data = JSON.parse(message.payloadString);
                } catch (e) {
                    data = message.payloadString;
                }
                this.onMessage(data, message.destinationName);
            } catch (error) {
                console.error("处理 MQTT 消息失败:", error);
            }
        }
    }

    handleReconnect() {
        this.reconnectAttempts++;
        // 使用退避策略，随着失败次数增加延长重连时间，最大 30 秒
        const delay = Math.min(this.reconnectInterval * Math.max(1, Math.floor(this.reconnectAttempts / 2)), 30000);

        console.log(`MQTT 连接丢失，将在 ${delay / 1000} 秒后尝试重新连接...`);

        setTimeout(() => {
            if (!this.client || !this.client.isConnected()) {
                this.connect();
            }
        }, delay);
    }

    /**
     * 发送消息
     * @param {string|object|Uint8Array} payload - 消息内容
     * @param {string} [topic] - 目标主题，默认为配置的主题
     */
    send(payload, topic) {
        if (this.client && this.client.isConnected()) {
            let message;
            if (payload instanceof Uint8Array) {
                message = new Paho.MQTT.Message(payload);
            } else if (typeof payload === "object") {
                message = new Paho.MQTT.Message(JSON.stringify(payload));
            } else {
                message = new Paho.MQTT.Message(String(payload));
            }

            message.destinationName = topic || this.config.topic;
            this.client.send(message);
        } else {
            console.warn("MQTT 未连接，无法发送消息");
        }
    }

    /**
     * 将 hex 字符串转换为字节数组并发送
     * @param {string} hexString
     */
    sendHex(hexString) {
        const hex = hexString.replace(/\s+/g, "");
        const bytes = new Uint8Array(hex.length / 2);
        for (let i = 0; i < hex.length; i += 2) {
            bytes[i / 2] = parseInt(hex.substr(i, 2), 16);
        }
        this.send(bytes);
    }
}
