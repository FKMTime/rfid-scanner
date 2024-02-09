<script lang="ts">
    let scannedCards: number[] = [];
    let serial: any | null = null;
    let lineBuffer: string = "";

    let lastDuplicated: boolean = false;

    async function getSerialPort() {
        //@ts-ignore
        serial = await navigator.serial.requestPort({
            filters: [{ usbVendorId: 0x2e8a }],
        });

        if (!serial) return;
        await serial.open({ baudRate: 9600 });

        while (serial.readable) {
            const reader = serial.readable.getReader();
            try {
                while (true) {
                    const { value, done } = await reader.read();
                    if (done) {
                        break;
                    }

                    // Decode the value into a string. if the value ends with /r/r then we have a complete line
                    let str = new TextDecoder().decode(value);
                    if (str.endsWith("\r\n")) {
                        lineBuffer += str;
                        let cardId = parseInt(lineBuffer.trim());
                        await scanCard(cardId);
                        lineBuffer = "";
                    } else {
                        lineBuffer += str;
                    }
                }
            } catch (error) {
            } finally {
                reader.releaseLock();
            }
        }
    }

    async function scanCard(cardId: number) {
        lastDuplicated = scannedCards.includes(cardId);

        if (scannedCards.includes(cardId)) {
            console.log(`Card ${cardId} already scanned!`);

            scannedCards = scannedCards.filter((c) => c !== cardId);
            scannedCards.push(cardId);
            scannedCards = scannedCards;

            return;
        }
        scannedCards.push(cardId);
        scannedCards = scannedCards;
        console.log(`Scanned card: ${cardId}, Total: ${scannedCards.length}`);
    }
</script>

<button on:click={getSerialPort}>Get Serial Port</button>

{#if serial}
    {#if scannedCards.length > 0}
        <h2>
            Last Scanned: {scannedCards[scannedCards.length - 1]}
            {lastDuplicated ? "(DUPLICATED)" : ""} | Total: {scannedCards.length}
        </h2>

        <h3>Scanned Cards</h3>
        <ul>
            {#each scannedCards as card}
                <li>{card}</li>
            {/each}
        </ul>
    {/if}
{/if}
